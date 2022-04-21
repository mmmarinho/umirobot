import requests
import hashlib
import random

from .teleop_configurator import *
from .teleop_tunnel import TeleopProxyTunnel


class Server:
    def __init__(self, server_key_path, ssh_key_path, team_name):
        """
        Create a server instance
        :param server_key_path: server password key file
        :param ssh_key_path: ssh private key path
        :param team_name: Unique team and forwarding instance identifier
        """
        self.server_key = read_server_key(server_key_path)
        self.ssh_key_path = ssh_key_path
        self.team_name = team_name
        self.opened_port = None
        self.tunnel_h = None

        self.session_identity = random.getrandbits(64)
        self.auth_payload = {'request_type': 'auth', 'session_id': self.session_identity}
        self.open_payload = {'request_type': 'open',
                             'session_id': self.session_identity,
                             'hash_secrete': "",
                             'team': hashlib.md5(self.team_name.encode('utf-8')).hexdigest()}

    def open(self, local_port, local_ip=None):
        """
        Open ssh tunnel
        :param local_port: local port number (port that vrep is listening)
        :param local_ip: local ip address (computer vrep is running). Default to loopback
        :return: None
        """
        auth_ret = requests.post(REQUEST_URL, data=self.auth_payload)
        auth_str = get_auth_string(auth_ret.text)
        if auth_str is None:
            raise RuntimeError("Initial authentication request to the server is not successful")

        hash_raw = "{}_{}_{}".format(self.session_identity,
                                     self.server_key,
                                     auth_str)
        auth_hash = hashlib.md5(hash_raw.encode('utf-8')).hexdigest()

        # send validate info and actual query request
        self.open_payload['hash_secrete'] = auth_hash
        open_ret = requests.post(REQUEST_URL, data=self.open_payload)

        self.opened_port = get_return_port(open_ret.text)
        if self.opened_port is None:
            raise RuntimeError("return port request is not successful")

        if local_ip is None:
            local_ip = "127.0.0.1"

        self.tunnel_h = TeleopProxyTunnel("guest", self.ssh_key_path,
                                          SERVER_LOCATION, self.opened_port,
                                          local_ip, local_port)

        self.tunnel_h.open()

        return SERVER_LOCATION, self.opened_port

    def is_alive(self):
        """
        Check if tunnel is alive
        :return: true if alive
        """
        return self.tunnel_h.is_alive()

    def close(self):
        """
        Close tunnel
        :return:
        """
        self.tunnel_h.close()
