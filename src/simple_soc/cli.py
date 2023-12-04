import argparse

from amaranth import cli

from .top import SimpleSoC


__all__ = ["main"]


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--rom-init",
            type=argparse.FileType("rb"), help="ROM init file")

    cli.main_parser(parser)

    args = parser.parse_args()

    rom_init = [int.from_bytes(w, "little") for w in iter(lambda: args.rom_init.read(4), b'')]
    top      = SimpleSoC(rom_init=rom_init)

    cli.main_runner(parser, args, top, name="top", ports=())


if __name__ == "__main__":
    main()
