# Lab Exercise 1: Nodes, Topics and Services

## Objective

In this lab, you will create and run basic ROS 2 publisher and subscriber nodes to understand how nodes communicate using topics. Then you will implement a simple service for request-response communication.

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Basic Python programming knowledge
- Terminal/command prompt access
- Completed Chapter 1 of this module

## Estimated Duration

45 minutes

## Instructions

### Part A: Running the Publisher-Subscriber Example

### Step 1: Set up your workspace
Open a terminal and create a ROS 2 workspace if you haven't already:
```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
```

### Step 2: Ensure environment is sourced
```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Run the publisher node
Open a new terminal, source the ROS 2 environment, and run the publisher:
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
python3 examples/publisher-example.py
```

### Step 4: Run the subscriber node
Open another terminal, source the ROS 2 environment, and run the subscriber:
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
python3 examples/subscriber-example.py
```

### Step 5: Observe the communication
You should see the publisher sending messages with an incrementing counter, and the subscriber receiving and displaying these messages. 

### Part B: Running the Service Example

### Step 6: Run the service server
In a new terminal, source the environment and run the service server:
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
python3 examples/service-server-example.py
```

### Step 7: Run the service client
In another terminal, source the environment and run the service client:
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
python3 examples/service-client-example.py
```

### Step 8: Observe the service communication
Notice how the client sends a request to the server and receives a response with the current time.

## Expected Output

### For Publisher-Subscriber:
- Publisher terminal: Should display "Publishing: 'Hello World: X'" every 0.5 seconds
- Subscriber terminal: Should display "I heard: 'Hello World: X'" in response to each message

### For Service:
- Service server terminal: Should log "Handled time request" when a client makes a request
- Service client terminal: Should display the response with the current time

## Validation Commands

After running the examples, use these commands to verify your setup:

```bash
# List active nodes
ros2 node list

# List active topics
ros2 topic list

# Echo messages on the chatter topic to see publisher-subscriber communication
ros2 topic echo /chatter std_msgs/msg/String

# List active services
ros2 service list

# Get info about the time service
ros2 service info /get_current_time
```

## Troubleshooting Tips

- If nodes don't communicate, ensure both terminals have the ROS 2 environment sourced
- If you get "module not found" errors, make sure you've installed ROS 2 properly
- If the service client doesn't work, ensure the server is running first
- Use `ros2 doctor` to diagnose common ROS 2 issues

## Extension Activities (Optional)

1. Modify the publisher to send different types of messages (e.g., integers instead of strings)
2. Create a second subscriber that listens to the same topic
3. Modify the service to accept parameters and return customized responses

## Assessment

Complete the following to verify your understanding:
1. Explain the difference between the publish-subscribe pattern and the client-server pattern
2. Identify the topic name used in the publisher-subscriber example
3. Describe what happens when you run multiple subscribers for the same topic