# Extra Content for the UMIRobot

|   |   |   | 
|---|---|---|
|umirobot_task_space_control|Moving your UMIRobot in task space using an constrained kinematic controller.| Murilo M. Marinho
|TeleopProxy|Create TCP Proxy Tunnel for teleoperation across WAN|Quentin Lin


___
## Usage:

### TeleopProxy

How to Use:
- Place `TeleopProxy` directory in the same directory of your code
- Get **Server Key** and **ssh Private Key** file from TA or Professor
- Agree on a unique **Team Name** with your partner

Server side example (on computer that is running Coppelia Sim)
```python
from TeleopProxy import Server as TeleopProxyServer
import time

server_key_path = "./client_secrete/server_key.key"  # location of the server key
ssh_key_path = "./client_secrete/id_rsa"             # location of the ssh private key
team_name = "TEAM1111"                               # pre-negotiated unique Team Name
vrep_port = 20000                                    # port that Coppelia Sim is listening

# start server instance
server = TeleopProxyServer(server_key_path, ssh_key_path, team_name)

# open tunnel
connect_info = server.open(vrep_port)

print("connection will be open at:", connect_info)

# Keep the script running to ensure subprocess stay alive
try:
    while True:
        time.sleep(1)
        # check if server is alive
        if server.is_alive():
            print("is_alive")
except KeyboardInterrupt:
    pass
except Exception as e:
    print("Unexpected Exception: " + str(e))

# close tunnel (Please ensure to do so properly)
server.close()
print("Server closed")
```

Client side example (on computer that is sending command to the robot)
```python
from TeleopProxy import Client as TeleopProxyClient

server_key_path = "./client_secrete/server_key.key"  # location of the server key
team_name = "TEAM1111"                               # pre-negotiated unique Team Name

# start client instance
client = TeleopProxyClient(server_key_path, team_name)

#quary for remote address and port
remote, remote_port = client.query()

print("Connection is opened at:", remote, remote_port)
```
or 
```python
from TeleopProxy import query_server_location

server_key_path = "./client_secrete/server_key.key"  # location of the server key
team_name = "TEAM1111"                               # pre-negotiated unique Team Name

#quary for remote address and port in single function
remote, remote_port = query_server_location(server_key_path, team_name)

print("Connection is opened at:", remote, remote_port)
```
## Combining with *umirobot-task-space-control*

For Server side (Side with physical robot)
- Add to the `import` section in `main.py` file

```python
from TeleopProxy import Server as TeleopProxyServer
server_key_path = "./client_secrete/server_key.key"  # location of the server key
ssh_key_path = "./client_secrete/id_rsa"             # location of the ssh private key
team_name = "TEAMxxxx"                               # pre-negotiated unique Team Name
vrep_port = 20000     
```

- Add to just after `with UMIRobot() as umirobot...` in `main.py` file

```python
server = TeleopProxyServer(server_key_path, ssh_key_path, team_name)
# open tunnel
connect_info = server.open(vrep_port)
print("connection will be open at:", connect_info)
```
- Add to just after `shared_memory_receiver_process.join()` in `main.py` file
  
  Same indent level with `if shared_memory_receiver_process.is_alive():`
```python
server.close()
```
