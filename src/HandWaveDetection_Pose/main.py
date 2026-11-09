from raised_hand.app import build_parser, resolve_config, run


def main() -> None:
    args = build_parser().parse_args()
    run(resolve_config(args))


if __name__ == '__main__':
    main()
