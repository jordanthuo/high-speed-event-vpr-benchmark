# VPR attribution

This rig-specific implementation follows the event-count representation, sum of absolute differences (L1/cityblock) matching, and fixed random-pixel baseline demonstrated in Tobias Fischer's sparse-event-vpr repository and OpenNeuromorphic tutorial. The rig adapter, centred angle-sampled extraction, circular ground truth, abstention policy, reporting and tests are new project code. This is not a reproduction of every component of the paper's sparse/varying-pixel method.

- Repository and tutorial: https://github.com/Tobias-Fischer/sparse-event-vpr
- Tutorial: https://github.com/Tobias-Fischer/sparse-event-vpr/blob/main/OpenNeuromorphicTalk.ipynb
- Core implementation: https://github.com/Tobias-Fischer/sparse-event-vpr/blob/main/src/sparse_event_vpr/utils.py
- Fischer, T. and Milford, M. (2022). *How Many Events Do You Need? Event-based Visual Place Recognition Using Sparse But Varying Pixels.* IEEE Robotics and Automation Letters, 7(4), 12275–12282. https://doi.org/10.1109/LRA.2022.3216226

Upstream license, retained for attribution:

MIT License

Copyright (c) 2022 Tobias Fischer

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
