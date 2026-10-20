# Project Proposal: Smart Warehouse Robot Simulator

**Course:** AL-2002 --- Artificial Intelligence Lab\
**Section:** BSCY - 5A\
**Date:** 20 October 2026\
**Project Repository:** https://github.com/anashaider/SmartWarehouseAI
**Project Type:** AI-Based Simulation

## 1. Project Title

**Smart Warehouse Robot Simulator**

## 2. Group Members

  Role           Name            Roll Number
  -------------- --------------- -------------
  Group Leader   Anas Haider     24K-2002
  Group Member   Shayan Hassan   24K-2050

## 3. Project Summary

The Smart Warehouse Robot Simulator is an AI-based simulation of a
warehouse in which multiple robots collect parcels from storage
locations and deliver them to designated drop-off points. The warehouse
is represented as a grid containing open paths, shelves, parcel
locations, and delivery points.

The project will demonstrate how artificial intelligence agents can
perceive their environment, plan routes, avoid obstacles, and complete
delivery tasks. Search algorithms such as Breadth-First Search (BFS) and
A\* will be implemented and compared using measurable performance
metrics.

## 4. Problem Statement

Warehouses require efficient movement of goods between storage and
delivery areas. A robot must find a valid route while avoiding shelves
and other robots. Different route-planning methods can produce different
results in terms of path length, explored nodes, and execution time.

This project will model a small warehouse and evaluate how different AI
agents and search algorithms perform in completing delivery tasks.

## 5. PEAS Description

  -----------------------------------------------------------------------
  Component                           Description
  ----------------------------------- -----------------------------------
  **Performance Measure**             Successful deliveries, total
                                      distance travelled, time/steps
                                      taken, nodes explored, and
                                      collision count

  **Environment**                     Grid-based warehouse with shelves,
                                      parcels, drop-off points, and
                                      multiple robots

  **Actuators**                       Move up, down, left, or right; pick
                                      up a parcel; deliver a parcel

  **Sensors**                         Current grid position, nearby
                                      obstacles, parcel locations,
                                      delivery points, and other robots'
                                      positions
  -----------------------------------------------------------------------

## 6. Environment and Agent Type

-   **Environment:** Grid-based, partially dynamic when robots move, and
    discrete.
-   **Observability:** The simulation will use a known grid map; robots
    will also account for occupied cells.
-   **Agent Types:** Simple Reflex, Goal-Based, and Utility-Based
    agents.
-   **Number of Robots:** Two to three robots, depending on the final
    implementation.
-   **Main Objective:** Deliver parcels safely while reducing total
    travel distance and avoiding collisions.

## 7. Problem Representation

The warehouse will be represented as a two-dimensional grid. Each cell
may represent an empty path, a shelf/obstacle, a parcel, or a delivery
point. A robot's state will include its current position and task
status. Legal actions will move the robot one cell in one of four
directions, provided the destination is valid and not blocked.

The goal test will check whether the assigned parcel has been delivered
to the required drop-off point. The path cost will primarily be based on
the number of moves, with collision avoidance handled as a constraint.

## 8. AI Techniques and Algorithms

### Search Algorithms

-   **Breadth-First Search (BFS):** Finds a shortest path in an
    unweighted grid and will be used as a baseline.
-   **A\* Search:** Uses path cost and a heuristic, such as Manhattan
    distance, to guide the search toward the goal.

### Intelligent Agents

-   **Simple Reflex Agent:** Selects actions using immediate rules and
    the current situation.
-   **Goal-Based Agent:** Chooses actions to reach a defined delivery
    goal.
-   **Utility-Based Agent:** Selects actions using a score that can
    consider distance, delivery progress, and collision risk.

### Multi-Robot Coordination

The simulation will prevent robots from occupying the same cell at the
same time where possible. The system will check occupied cells and
handle conflicts by waiting or replanning a route.

## 9. Object-Oriented Design

The project will use object-oriented programming with at least the
following classes:

-   **Environment:** Creates and manages the warehouse grid, obstacles,
    parcels, and delivery locations.
-   **RobotAgent:** Stores a robot's position, assigned task, and
    actions.
-   **SearchAlgorithms:** Implements BFS and A\* pathfinding.
-   **WarehouseSimulator:** Runs the simulation, coordinates robots, and
    records performance metrics.

The class structure may be adjusted during implementation while
maintaining clear responsibilities and readable code.

## 10. Tools and Technologies

-   **Programming Language:** Python
-   **Development Environment:** Visual Studio Code
-   **Visualization:** Pygame
-   **Charts and Analysis:** Matplotlib
-   **Version Control:** Git and GitHub

The project will use a simulated warehouse environment, so no external
or private dataset is required. A fixed random seed will be used for
reproducible generated layouts and experiments.

## 11. Project Scope

### Included

-   Grid-based warehouse with shelves, parcel locations, and drop-off
    points.
-   Two to three robots completing delivery tasks.
-   BFS and A\* implemented and compared.
-   Simple Reflex, Goal-Based, and Utility-Based agent behaviour.
-   Basic collision avoidance and route replanning or waiting.
-   Simulation dashboard or visual display of robot movement and
    deliveries.
-   Performance measurements and charts.

### Not Included

-   Physical robot hardware.
-   Real warehouse integration or live sensor data.
-   Large-scale industrial fleet management.
-   Advanced machine-learning model training.

## 12. Evaluation Plan

The project will compare BFS and A\* on the same warehouse layouts and
task configurations. Experiments will use fixed seeds so the results can
be reproduced.

The following metrics will be recorded: 1. Number of successful
deliveries. 2. Total distance or number of moves. 3. Number of nodes
explored by the search algorithm. 4. Execution time. 5. Number of
collisions or movement conflicts.

At least three charts will be produced, for example: - BFS versus A\*
path length. - BFS versus A\* nodes explored. - Delivery performance or
execution time across test scenarios.

The results will be discussed against the BFS baseline. Any limitations
or unsuccessful cases will be documented honestly.

## 13. Work Division

  -----------------------------------------------------------------------
  Member                              Responsibilities
  ----------------------------------- -----------------------------------
  **Anas Haider (24K-2002)**          Project coordination, warehouse
                                      environment, simulator integration,
                                      testing, and final documentation

  **Shayan Hassan (24K-2050)**        Search algorithm implementation,
                                      agent logic, performance
                                      measurements, charts, and testing
  -----------------------------------------------------------------------

Both members will contribute code and documentation and will make their
own meaningful commits to the public GitHub repository.

## 14. Project Timeline

  -----------------------------------------------------------------------
  Phase                               Planned Work
  ----------------------------------- -----------------------------------
  Phase 1                             Set up the repository, define the
                                      grid environment, and prepare the
                                      project structure

  Phase 2                             Implement robot movement, goals,
                                      and BFS

  Phase 3                             Implement A\* and agent behaviours

  Phase 4                             Add multi-robot coordination and
                                      collision handling

  Phase 5                             Run reproducible experiments and
                                      generate at least three charts

  Phase 6                             Test from a clean clone, complete
                                      README and requirements, and
                                      prepare the final demonstration
  -----------------------------------------------------------------------

## 15. Expected Outcomes

By the end of the project, the team expects to deliver: - A working
visual warehouse robot simulator. - BFS and A\* pathfinding
implementations. - Multiple agent behaviours and basic multi-robot
coordination. - Reproducible experimental results and comparison
charts. - A public GitHub repository with source code, README,
requirements, and project documentation.

## 16. Risks and Mitigation

  -----------------------------------------------------------------------
  Potential Risk                      Mitigation
  ----------------------------------- -----------------------------------
  Robots block one another            Check occupied cells and use
                                      waiting or replanning

  Search algorithms behave            Test on small, known grid layouts
  incorrectly                         before larger experiments

  Visualization takes too much time   Complete core pathfinding and
                                      simulation logic before polishing
                                      the interface

  Results are difficult to reproduce  Use fixed random seeds and document
                                      test configurations
  -----------------------------------------------------------------------

## 17. Repository

**GitHub:** https://github.com/anashaider/SmartWarehouseAI

The repository will contain the source code, `README.md`,
`requirements.txt`, and this proposal under the `docs/` directory.
