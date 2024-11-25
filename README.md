![banner](https://github.com/user-attachments/assets/89325875-c7e3-45df-8fd2-01137fd0d1c4)

# Self-Aligning Satellite Dish
## Table of Contents
- [Introduction](https://github.com/ian-ndeda/self-aligning-sat-dish/tree/main?tab=readme-ov-file#introduction)
- [Flowchart](https://github.com/ian-ndeda/self-aligning-sat-dish/tree/main?tab=readme-ov-file#flowchart)
- [Requirements](https://github.com/ian-ndeda/self-aligning-sat-dish/tree/main?tab=readme-ov-file.md#requirements)
- [Connections](https://github.com/ian-ndeda/self-aligning-sat-dish/tree/main?tab=readme-ov-file#connections)
- [Serial Bluetooth App](https://github.com/ian-ndeda/self-aligning-sat-dish/tree/main?tab=readme-ov-file#serial-bluetooth-app)
- [Results](https://github.com/ian-ndeda/self-aligning-sat-dish/tree/main?tab=readme-ov-file#results)
- [Recommndations](https://github.com/ian-ndeda/self-aligning-sat-dish/tree/main?tab=readme-ov-file.md#recommendations)


## Introduction

Rust is a relatively new programming language with a lot of potential in systems programming. Its memory safety makes it a particularly useful choice in building secure software.

In this project, we are going to build a simple model for a self-aligning satellite dish, i.e., a dish that tracks a selected satellite in the geostationary orbit.

We will be using Rust to program a Raspberry Pi Pico microcontroller. Beginner-level Rust competency is therefore necessary. The [Rust Book](https://doc.rust-lang.org/book/title-page.html) is an excellent resource for both learning and as a reference.

>💡 This is a summary of a tutorial first published [here](https://dev.to/ian_ndeda/self-aligning-dish-in-rust-introduction-1e74).


## Flowchart

We want to be able to operate the dish system in either auto or manual modes. In auto mode, the dish should seek the satellite out and align with it automatically, with any deviation immediately corrected.

In manual mode, we should be able to align the dish on our own by panning and tilting it to face our desired direction. This functionality can be useful when you need to fine-tune the system after an initial auto-alignment.

The general flow of our program will be fairly simple, as shown in the following flowchart:

<p align="center">
  <img src="https://github.com/user-attachments/assets/434dba53-6d75-42d5-85d4-f6d854ad9840" width="600" height="600">
</p>

In our model, we will use a pan-tilt (PTZ) kit to simulate the dish's movements. The kit will be actuated using servo motors. We will also use an electronic compass to get the magnetic headings, a GPS module to acquire coordinates, and a Bluetooth module to communicate remotely with our system.

We have to accomplish the following for our project to work:
- Listen for commands
- Acquire GPS coordinates
- Calculate look angles
- Align the dish and
- Blink every second to indicate the program is running.


## Requirements

- 1 x Raspberry Pico board
- 1 x USB Cable type 2.0
- 1 x HC-05 Bluetooth module
- 1 x HMC5833L Compass Module
- 40 x M-M Jumper Wires 
- 2 x Mini Breadboards
- 2 x SG90 Servo Motor
- 1 x PTZ Kit
- 1 x GPS Module
- 2 x Relay modules 
- 2 x 4.2V Li-ion Batteries
- 1 x DC-DC Step-down Converter
- 1 x Diode
- Serial Bluetooth App


## Connections
Connect the components as shown below.

<p align="center">
  <img src="https://github.com/user-attachments/assets/93fa38dd-83ef-40bb-905c-6975bcfbd903" width="600" height="600">
</p>

The compass/magnetometer is mounted on the the face of the PTZ kit as shown below.

<p align="center">
  <img src="https://github.com/user-attachments/assets/7ece4967-7fb6-4732-a8d7-42699aea6a20" width="350" height="350">
</p>

## Serial Bluetooth App

We'll use the [Serial Bluetooth App](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=en) to send commands to our system remotely. 

We have to set up the app so that it sends precisely one byte per press. Navigate to settings and under `Newline` select `None`.

We will also allow local echo to help us visualize the commands we are sending.


<p align="center">
  <img src="https://github.com/user-attachments/assets/354f88b9-001c-4b63-bfe1-c36fd44984a2" width="370" height="800">
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/1dfae6fc-2a30-469a-8c4b-b5ed0c936623" width="370" height="800">
</p>

Edit the macro rows at the bottom to have the commands discussed above i.e. `Auto/Manual`, `CW`, `CCW`, `UP`, `DOWN` and `Zero`.

You should have something of this sort:

<p align="center">
  <img src="https://github.com/user-attachments/assets/273d1445-790c-47ba-ae3b-eb8a6ddc8369" width="370" height="800">
</p>


## Results

The application code is derived from the algorithm below.

<p align="center">
  <img src="https://github.com/user-attachments/assets/f6c574f9-e418-4aab-b66f-df2440824b5f" width="700" height="700">
</p>

The final code of the project can be found [here](https://github.com/ian-ndeda/self-aligning-sat-dish/blob/main/snapshots/final.rs).

Load the above program into the Pico in `release` mode.

```rust
cargo run --release
```

> **NB**❗ The Pico will need to be in flash mode. This is achieved by plugging the USB into the computer while the button on the Pico is pressed.

While in manual mode, giving the `CW` command has an appropriate effect on the value of the compass heading; the same with the `CCW` command. The `position zero` command aligns the system to generally face North.

<p align="center">
  <img src="https://github.com/user-attachments/assets/e07c63a9-f3bd-478c-ace8-1c6263fa6a94" width="370" height="800">
</p>

While in auto mode, the kit aligns to the look angles, starting with a pan until `theta` is achieved and then a tilt up to `phi`.

<p align="center">
  <img src="https://github.com/user-attachments/assets/e07c63a9-f3bd-478c-ace8-1c6263fa6a94" width="370" height="800">
</p>

Below is a simple demonstration where we see the PTZ kit in auto mode tracking particular look angles when the whole system is rotated.

<p align="center">
  <img src="https://github.com/user-attachments/assets/a170e55f-f360-4276-8ee5-5aca8773f26b" width="350" height="350">
</p>


## Recommendations


This project took a naive, straightforward approach in fulfilling its objective. There is a lot of room for improvement. Some low-lying picks are listed below:

- Introduce the RP2040's watchdog timer.Some of the code we wrote in the project is "blocking," which means that subsequent code must wait until it has finished running. Sometimes these code segments might not finish, freezing the software as a whole. We could use Async Rust or Rust's RTIC framework to write alternative non-blocking programs. We could also use a watchdog timer to break out of these extended loops.
- Have more functions returning. In case of an error during a run, the function will simply return the error for handling instead of stalling the entire system.
- Use a gyroscope to complement the magnetometer. This will enable the acquisition of tilt-compensated compass headings. Check [this](https://www.instructables.com/Tilt-Compensated-Compass/).
- Implement the entire program as a state machine. 
- Produce an electrical schematic and PCB for the project.
