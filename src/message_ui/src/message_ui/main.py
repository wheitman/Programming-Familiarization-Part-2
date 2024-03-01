import sys

from rqt_gui.main import Main


def main():
    main = Main()
    sys.exit(main.main(sys.argv, standalone="message_ui.message_ui.MessageUI"))


if __name__ == "__main__":
    main()
