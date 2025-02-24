# Mish: MAgIC Robotic Fish

## Introduction

**Mish** is a three joints robotic fish prototype, mimiced a typical carangiform fish with Body and/or Caudal Fin (BCF) propulsion mode, enabling agile planar movement near the water surface. A key feature distinguishing  **Mish** from others is its capability to provide real-time joint position feedback, enhancing state representations for the controller, which is beneficial for generating multi-model swimming policy based on learning-based method, such as end-to-end Reinforcement Learning (RL).

<img src="" alt="" width="auto" height="400" />

This repository aims to provide comprehensive details of **Mish**, facilitating the creation of similar models by enthusiasts. We hope **Mish** can contribute to the development of the robotic fish community and support related scientific research work. 

If you have any questions, please feel free to raise an ISSUE. I am happy to help you reproduce your own **Mish**.

## Demonstration

**Mish** currently supports wireless control of the desired angle position of joints through UDP protocol at a maximum frequency of 50Hz. Before running the python script in upper computer, please ensure that the **Mish** and the upper computer are on the same WIFI network and check if the IP address and port are correct.

By running the ```demonstration.py``` script, it can be observed that the robotic fish swings its joints with sine signal.

<div align="center">
  <video width="640" height="" controls>
    <source src="" type="video/mp4">
  </video>
</div>

## Production

**Mish**'s dimensions are 0.52m * 0.13m * 0.12 m, weighing 1.1 kg. The prototype is comprised of three main components: (i) a rigid head housing an MCU board with the ESP32-S3 module and an 800 mAh 7.4 V lithium battery; (ii) a deformable body consisting of three joints connected by aluminum components and covered with rubber skin; and (iii) a rigid tail.

<div align="center">
  <img src="./00 docs/assembly.jpg" alt="" width="45%" height="" />
  <img src="./00 docs/disassembly.jpg" alt="" width="45%" height="" />
</div>

The head and tail are 3D-printed from ABS plastic (PLA is even better) and coated with epoxy resin for waterproofing. Each joint is driven by a Hiwonder LX-824 servomotor with a maximum torque of 17 kg*cm, a no-load speed of 0.2 second per 60 degree. The density of the prototype is adjusted slightly lower than water to focus on its planar motion.

All the production materials for **Mish** have been provided, please refer to [docs](./00%20docs/), [model](./01%20model/), [hardware](./02%20hardware/) and [firmware](./03%20firmware/) for more details.

## Acknowledgment

The author is very grateful for the help of Yangyang Zhao, Hongru Dai, Xu Huang and Yuhang Zhao from the [MAgIC Lab](https://magiclab.sist.shanghaitech.edu.cn) at [ShanghaiTech Univeristy](https://www.shanghaitech.edu.cn/eng/).

## References

0. [Learning Agile Swimming: An End-to-End Approach Without CPGs](https://arxiv.org/abs/2409.10019)
1. [CPG-Based Locomotion Control Of A Robotic Fish: Using Linear Oscillators And Reducing Control Parameters Via PSO](http://www.ijicic.org/10-05048-1.pdf)
2. [Bottom-Level Motion Control For Robotic Fish To Swim In Groups: Modeling And Experiments](https://iopscience.iop.org/article/10.1088/1748-3190/ab1052)
3. [A General CPG Network And Its Implementation On The Microcontroller](https://www.sciencedirect.com/science/article/abs/pii/S0925231215005354)

## BibTeX

If you think this repository is helpful to you, please consider citing our latest work.

```bash
@article{lin2025learning,
  title={Learning Agile Swimming: An End-to-End Approach Without CPGs},
  author={Lin, Xiaozhu and Liu, Xiaopei and Wang, Yang},
  journal={IEEE Robotics and Automation Letters},
  year={2025},
  publisher={IEEE}
}
```
