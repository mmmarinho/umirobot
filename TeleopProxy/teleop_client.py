import requests
import hashlib
import random

from .teleop_configurator import *


def get_remote():
    return SERVER_LOCATION


class Client:
    def __init__(self, server_key_path, team_name):
        """
        Create a client instance
        :param server_key_path: server password key file
        :param team_name: Unique team and forwarding instance identifier
        """
        self.server_key = read_server_key(server_key_path)
        self.team_name = team_name
        self.opened_port = None

        self.session_identity = random.getrandbits(64)
        self.auth_payload = {'request_type': 'auth', 'session_id': self.session_identity}
        self.request_payload = {'request_type': 'request',
                                'session_id': self.session_identity,
                                'hash_secrete': "",
                                'team': hashlib.md5(self.team_name.encode('utf-8')).hexdigest()}

    def query(self):
        """
        Query for the port and remote information with the given team name
        :return: remote, remote_port
            remote address, forwarded port
            remote_port==None if team_name is not found on server
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
        self.request_payload['hash_secrete'] = auth_hash
        open_ret = requests.post(REQUEST_URL, data=self.request_payload)

        self.opened_port = get_return_port(open_ret.text)
        if self.opened_port is None:
            return SERVER_LOCATION, None

        return SERVER_LOCATION, self.opened_port

    def get_port(self):
        return self.opened_port

