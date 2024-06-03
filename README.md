# BBRv3 in ns-3

## Overview of BBRv3

BBRv3 uses a model-based approach to congestion control, estimating the bottleneck bandwidth and the round-trip propagation time in the network. It adjusts the sending rate based on these estimates to achieve high throughput and low latency, avoiding the traditional loss-based congestion control mechanisms. 


## Integration in ns-3

In ns-3, BBRv3 is implemented within the internet module. The core implementation can be found in the following files:

- [`tcp-bbrv3.cc`](src/internet/model/tcp-bbr3.cc): This file contains the main implementation of the BBRv3 algorithm, including the logic for estimating network parameters and adjusting the sending rate.
- [`tcp-bbrv3.h`](src/internet/model/tcp-bbr3.h): This header file defines the classes and functions used in the BBRv3 implementation.

### File Locations

- [tcp-bbrv3.cc](src/internet/model/tcp-bbr3.cc)
- [tcp-bbrv3.h](src/internet/model/tcp-bbr3.h)

## How to Use BBRv3 in ns-3

To use BBRv3 in your ns-3 simulations, follow these steps:

1. Run the [`run.sh`](run.sh) script which will copy over the necessary files and then build your ns-3 using the optimized profile 

2. **Include BBRv3 in your simulation script**: Modify your simulation script to use "TcpBbr3" as the congestion control algorithm. 
