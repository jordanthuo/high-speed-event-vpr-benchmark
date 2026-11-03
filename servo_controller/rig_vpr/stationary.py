"""Pre-motion event activity and candidate flicker frequencies from original RAW."""
from __future__ import annotations

import json
from datetime import datetime,timezone
from pathlib import Path
import numpy as np

from . import VERSION
from .algorithm import EventSource,code_fingerprints,runtime_versions
from .data import resolve,sha256
from .inspection import completed_source,frozen_trace,verify_raw
from .reporting import plotting,save_figure,write_csv


def activity_spectrum(counts,bin_us):
    """One-sided Hann-window PSD after linear detrending; candidates, not a cause test."""
    counts=np.asarray(counts,dtype=float)
    if len(counts)<16 or not np.all(np.isfinite(counts)):
        raise ValueError("At least 16 finite temporal bins are required")
    fs=1e6/bin_us
    t=np.arange(len(counts),dtype=float)
    rate=counts*fs
    trend=np.polyval(np.polyfit(t,rate,1),t)
    window=np.hanning(len(counts))
    power=abs(np.fft.rfft((rate-trend)*window))**2/(fs*np.sum(window**2))
    if np.max(abs(rate-trend))<=max(1,float(abs(rate).max()))*1e-10:
        power[:]=0
    if len(counts)%2==0:
        power[1:-1]*=2
    else:
        power[1:]*=2
    frequency=np.fft.rfftfreq(len(counts),d=1/fs)
    band=(frequency>=10)&(frequency<=min(500,fs/2))
    peak=None
    if band.any() and power[band].max()>1e-20:
        index=np.flatnonzero(band)[np.argmax(power[band])]
        peak=dict(strongest_peak_hz=float(frequency[index]),peak_power=float(power[index]),
            peak_to_median_band_power=float(power[index]/max(float(np.median(power[band])),1e-30)))
    fractions={}
    resolution=fs/len(counts)
    total=float(power[band].sum())
    for hz in (50,60,100,120):
        selected=band&(abs(frequency-hz)<=max(2,resolution))
        fractions[f"power_fraction_near_{hz}hz"]=float(power[selected].sum()/total) if total>1e-20 else None
    return frequency,power,dict(frequency_resolution_hz=resolution,**(peak or dict(strongest_peak_hz=None,peak_power=0,peak_to_median_band_power=None)),**fractions)


def stationary_plan(source,duration_s,guard_s,startup_s,bin_ms):
    values=(duration_s,guard_s,startup_s,bin_ms)
    if not all(np.isfinite(v) for v in values) or duration_s<=0 or guard_s<0 or startup_s<0 or bin_ms<=0:
        raise ValueError("Use positive duration/bin width and nonnegative guard/startup times")
    duration_us=round(duration_s*1e6);bin_us=round(bin_ms*1000)
    if bin_us<1 or duration_us%bin_us or duration_us//bin_us<16:
        raise ValueError("Duration must contain at least 16 whole temporal bins")
    if bin_us>1000:
        raise ValueError("Use bins <=1 ms to inspect candidate flicker up to 500 Hz")
    planned=[];traces={};sources={}
    for entry in source["protocol"]["acquisitions"]:
        if entry["sync_mode"]!="manual" or entry["manual_evk4_motion_s"] is None:
            raise ValueError("Stationary interval selection currently requires a frozen manual motion onset")
        end=round(entry["manual_evk4_motion_s"]*1e6)-round(guard_s*1e6)
        start=end-duration_us
        if start<round(startup_s*1e6) or end>round(entry["studio_duration_s"]*1e6):
            raise ValueError(f"{entry['key']}: insufficient guarded pre-motion interval; reduce duration or startup exclusion")
        trace,inputs=frozen_trace(source,entry)
        traces[entry["key"]]=trace;sources[entry["key"]]=inputs
        overlap=max(0.,min(end/1e6,trace["times"][-1])-max(start/1e6,trace["times"][0]))
        covered=bool(trace["times"][0]<=start/1e6 and end/1e6<=trace["times"][-1])
        maximum=None;travel=None
        if covered:
            times=np.r_[start/1e6,trace["times"][(trace["times"]>start/1e6)&(trace["times"]<end/1e6)],end/1e6]
            maximum=float(abs(np.interp(times,trace["times"],trace["speed"])).max())
            angle=np.interp(times,trace["times"],trace["angles"]);travel=float(angle.max()-angle.min())
            if maximum>.05 or travel>.1:
                raise ValueError(f"{entry['key']}: encoder-covered pre-motion interval is not stationary")
        planned.append(dict(acquisition=entry["key"] ,condition=entry["condition"],run=entry["run"],start_us=start,end_us=end,
            duration_s=duration_us/1e6,bin_us=bin_us,manual_motion_onset_s=entry["manual_evk4_motion_s"],guard_s=guard_s,
            encoder_coverage_fraction=overlap/(duration_us/1e6),encoder_fully_covers_interval=covered,
            maximum_abs_platform_rpm=maximum,encoder_angle_range_deg=travel,
            stationarity_basis="encoder_checked_and_manual_onset" if covered else "guarded_manual_onset_not_fully_encoder_verified"))
    return planned,sources


def analyse_stream(stream,start_us,end_us,bin_us):
    sw,sh=stream.sensor_size;n=(end_us-start_us)//bin_us
    counts=np.zeros((2,n),dtype=np.int64);maps=np.zeros((2,sh*sw),dtype=np.int64)
    origin=stream.origin_us
    # Bounded event arrays even for high stationary activity; timestamps define bins.
    for a in range(start_us,end_us,10_000):
        b=min(a+10_000,end_us);events=stream.events_between(a,b)
        if not len(events):
            continue
        if events.ndim!=2 or events.shape[1]!=4 or not np.all(np.isfinite(events)):
            raise ValueError("Invalid pre-motion events")
        x,y,t,p=events.T
        if np.any(x<0) or np.any(x>=sw) or np.any(y<0) or np.any(y>=sh) or not np.all(np.isin(p,[0,1])) or np.any(x!=np.floor(x)) or np.any(y!=np.floor(y)):
            raise ValueError("Invalid pre-motion event coordinates/polarity")
        bins=np.floor((t-origin-start_us)/bin_us).astype(np.int64)
        if np.any(bins<0) or np.any(bins>=n):
            raise ValueError("Pre-motion events fall outside the selected interval")
        polarity=p.astype(np.int64);pixel=y.astype(np.int64)*sw+x.astype(np.int64)
        counts+=np.bincount(polarity*n+bins,minlength=2*n).reshape(2,n)
        maps+=np.bincount(polarity*sw*sh+pixel,minlength=2*sw*sh).reshape(2,sw*sh)
    return counts,maps.reshape(2,sh,sw)


def stationary_figures(output,series):
    plt=plotting()
    map_max=max(float(np.log10(1+item["map"]).max()) for item in series)
    for name in ("event_rate","spectra","spatial_activity"):
        fig,axes=plt.subplots(3,3,figsize=(12,8),layout="constrained")
        for ax,item in zip(axes.flat,series):
            ax.set_title(item["key"],fontsize=10)
            if name=="event_rate":
                ax.plot(item["time_s"],item["counts"].sum(axis=0)*1e6/item["bin_us"]/1e6,color="#0072B2",linewidth=.6)
                ax.set(xlabel="Time in selected interval (s)",ylabel="Recorded rate (Mevents/s)")
            elif name=="spectra":
                for polarity,color,label in ((None,"black","All"),(1,"#D55E00","ON"),(0,"#0072B2","OFF")):
                    c=item["counts"].sum(axis=0) if polarity is None else item["counts"][polarity]
                    f,p,stats=activity_spectrum(c,item["bin_us"])
                    selected=(f>=10)&(f<=500);total=p[selected].max()
                    ax.plot(f[selected],p[selected]/max(total,1e-30),color=color,label=label,linewidth=.8)
                ax.set(xlabel="Frequency (Hz)",ylabel="Relative PSD",xlim=(10,500))
                for hz in (50,60,100,120):
                    ax.axvline(hz,color="gray",alpha=.2,linewidth=.5)
            else:
                im=ax.imshow(np.log10(1+item["map"]),cmap="magma",vmin=0,vmax=map_max or 1,interpolation="nearest")
                ax.set(xlabel="Binned sensor x",ylabel="Binned sensor y")
                fig.colorbar(im,ax=ax,fraction=.035,pad=.02,label="log10(1 + events/pixel/s)")
        if name=="spectra":
            axes.flat[0].legend(fontsize=7)
        fig.suptitle("Guarded pre-motion activity: "+name.replace("_"," ")+" (candidate periodicity is not proof of light flicker)",fontsize=12)
        save_figure(plt,fig,output/"figures"/name)


def stationary_analysis(source_results,duration_s=.5,guard_s=.2,startup_s=.2,bin_ms=.5,output_root=None,prepare_only=False,no_plots=False):
    source=completed_source(source_results)
    planned,inputs=stationary_plan(source,duration_s,guard_s,startup_s,bin_ms)
    output=Path(output_root).expanduser().resolve() if output_root else source["path"].parent.with_name("VPR-Stationary")
    if output==source["path"].parent or source["path"].parent in output.parents or output in source["path"].parent.parents:
        raise ValueError("Stationary output must be separate from original results/cache")
    settings=dict(source_protocol_sha256=source["complete"]["protocol_sha256"],duration_s=duration_s,guard_s=guard_s,startup_s=startup_s,bin_ms=bin_ms,planned=planned)
    if output.exists() and any(output.iterdir()):
        if not (output/"stationary_plan.json").is_file() or json.loads((output/"stationary_plan.json").read_text(encoding="utf-8"))!=settings:
            raise FileExistsError("Stationary output is not empty or settings changed; choose another output root")
        if (output/"stationary_complete.json").exists():
            raise FileExistsError("Stationary analysis already completed; use another output root")
    output.mkdir(parents=True,exist_ok=True)
    (output/"stationary_plan.json").write_text(json.dumps(settings,indent=2),encoding="utf-8")
    write_csv(output/"planned_intervals.csv",planned)
    if prepare_only:
        print(f"[PREPARED ONLY] {len(planned)} pre-motion intervals; no RAW analysed: {output}",flush=True);return output
    raw_info={entry["key"]:verify_raw(source,entry) for entry in source["protocol"]["acquisitions"]}
    if not no_plots:
        plotting()
    summaries=[];temporal=[];spectra=[];peaks=[];series=[];audit={}
    for entry,plan in zip(source["protocol"]["acquisitions"],planned):
        key=entry["key"];stream=EventSource(raw_info[key][0],entry["studio_duration_s"])
        try:
            if stream.origin_us!=source["audit"]["inputs"][key]["raw_origin_us"]:
                raise ValueError(f"{key}: RAW time origin changed")
            counts,maps=analyse_stream(stream,plan["start_us"],plan["end_us"],plan["bin_us"])
            backend=stream.backend
        finally:
            stream.close()
        total=counts.sum(axis=0);pixels=maps.sum(axis=0);count=int(total.sum());n=len(total);duration=plan["duration_s"]
        camera=json.loads(resolve(source["root"],entry["camera_metadata"]).read_text(encoding="utf-8"))
        summary=dict(**plan,total_events=count,events_per_second=count/duration,on_fraction=float(counts[1].sum()/count) if count else None,
            recorded_biases=json.dumps(camera.get("applied_biases"),sort_keys=True),
            recorded_camera_healthy_at_finalize=camera.get("camera_healthy_at_finalize"),
            active_pixel_fraction=float(np.mean(pixels>0)),empty_temporal_bin_fraction=float(np.mean(total==0)),
            top_1percent_pixel_event_fraction=float(np.sort(pixels.ravel())[-max(1,round(pixels.size*.01)):].sum()/count) if count else None)
        for label,c in (("all",total),("on",counts[1]),("off",counts[0])):
            f,p,stats=activity_spectrum(c,plan["bin_us"])
            summary.update({label+"_"+k:v for k,v in stats.items()})
            spectra.extend(dict(acquisition=key,polarity=label,frequency_hz=float(hz),power_events_per_second_squared_per_hz=float(power)) for hz,power in zip(f,p))
            candidate=(f>=10)&(f<=500)&(p>1e-20)&(p>=np.r_[0,p[:-1]])&(p>np.r_[p[1:],0])
            candidates=np.flatnonzero(candidate)
            candidates=candidates[np.argsort(-p[candidates],kind="stable")][:8]
            peaks.extend(dict(acquisition=key,polarity=label,rank=rank+1,frequency_hz=float(f[index]),power=float(p[index])) for rank,index in enumerate(candidates))
        summaries.append(summary)
        for i in range(n):
            temporal.append(dict(acquisition=key,bin_start_camera_s=(plan["start_us"]+i*plan["bin_us"])/1e6,
                interval_time_s=i*plan["bin_us"]/1e6,on_events=int(counts[1,i]),off_events=int(counts[0,i]),total_events=int(total[i])))
        path=output/"activity_maps"/(key.replace('/','_')+".npz");path.parent.mkdir(exist_ok=True)
        np.savez_compressed(path,on_counts=maps[1],off_counts=maps[0],duration_s=duration)
        # Small display image of mean native-pixel rate, not an additional event filter.
        h,w=pixels.shape;yy,xx=np.indices((h,w));index=(yy*90//h)*160+(xx*160//w)
        summed=np.bincount(index.ravel(),weights=pixels.ravel(),minlength=90*160)
        population=np.bincount(index.ravel(),minlength=90*160)
        display=(summed/np.maximum(population,1)/duration).reshape(90,160)
        series.append(dict(key=key,time_s=np.arange(n)*plan["bin_us"]/1e6,counts=counts,bin_us=plan["bin_us"],map=display))
        audit[key]=dict(raw=raw_info[key][1],inputs=inputs[key],event_source_backend=backend,recorded_camera_metadata=camera,activity_map_sha256=sha256(path))
        print(f"[STATIONARY] {key}: {count/duration/1e6:.3f} Mevents/s; strongest all-event peak {summary['all_strongest_peak_hz']}",flush=True)
    write_csv(output/"stationary_summary.csv",summaries);write_csv(output/"temporal_counts.csv",temporal);write_csv(output/"spectra.csv",spectra);write_csv(output/"spectral_peaks.csv",peaks)
    if not no_plots:
        stationary_figures(output,series)
    text=["# Guarded pre-motion activity","",
        "All nine intervals are selected using the accepted manual camera-motion onset, with the configured startup exclusion and pre-onset guard. Encoder coverage and the basis for stationarity are reported per acquisition. If the encoder does not cover an interval, stationarity is inferred from the accepted manual onset rather than independently encoder-verified.","",
        "Stationary activity is not automatically sensor noise. It can include illumination flicker, residual motion, electronic effects, hot pixels and genuine background activity. Spectral peaks are candidate periodicities, not proof of lighting flicker. PSDs are linearly detrended Hann-window estimates; frequency resolution is 1/interval duration. Examine ON and OFF as well as the total event stream, since flicker can produce harmonics.","",
        "Low/Medium/High labels identify the subsequent motion acquisition; they do not indicate platform movement during these pre-motion intervals. Recorded biases and camera health are retained, but a health flag is not evidence of lossless capture and bias values alone do not establish camera-side filtering/ERC state.","",
        "The source protocol supplies frozen onsets and RAW identities. Recorded telemetry and camera metadata are hash checked. Current setup/catalogue onsets are not substituted, and no preparation files are modified. RAW identity checks are size plus first/last MiB fingerprints, not whole-file checksums.",""]
    (output/"README.md").write_text("\n".join(text),encoding="utf-8")
    (output/"stationary_provenance.json").write_text(json.dumps(dict(version=VERSION,settings=settings,inputs=audit,code_sha256=code_fingerprints(),runtime=runtime_versions()),indent=2),encoding="utf-8")
    (output/"stationary_complete.json").write_text(json.dumps(dict(completed_utc=datetime.now(timezone.utc).isoformat(),acquisitions=len(summaries)),indent=2),encoding="utf-8")
    print(f"[DONE] Pre-motion analysis: {output}",flush=True);return output
