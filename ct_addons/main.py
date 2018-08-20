import argparse
from .transport import CTSocketClient
from .event_trackers import mock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--client-id', required=True)

    subparsers = parser.add_subparsers()

    mock_parser = subparsers.add_parser('mock')
    mock_parser.add_argument('--period', type=float, default=30.0)
    mock_parser.set_defaults(
        get_tracker=lambda client, args: mock.MockEventTracker(client, args.period),
        etype=mock.MockEventTracker.EVENT_TYPE,
    )

    args = parser.parse_args()

    def on_config_enabled(etype, params):
        tracker.on_config_enabled(etype, params)

    def on_config_disabled(etype, params):
        tracker.on_config_enabled(etype, params)

    client = CTSocketClient(args.client_id, [args.etype],
                            on_config_enabled, on_config_disabled)

    tracker = args.get_tracker(client, args)
    client.start()
    try:
        tracker.run()
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
