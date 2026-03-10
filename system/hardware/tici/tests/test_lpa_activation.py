import pytest

from openpilot.system.hardware.tici.lpa import parse_lpa_activation_code
from openpilot.system.hardware.tici.qr_decode import validate_lpa_activation_code


def test_parse_valid_activation_code():
  version, smdp, matching = parse_lpa_activation_code("LPA:1$rsp.truphone.com$QRF-BETTERROAMING")
  assert version == "1"
  assert smdp == "rsp.truphone.com"
  assert matching == "QRF-BETTERROAMING"


@pytest.mark.parametrize("code", [
  "",
  "foo",
  "LPA:2$rsp.truphone.com$abc",
  "LPA:1$$abc",
  "LPA:1$rsp.truphone.com$",
  "LPA:1$rsp.truphone.com",
])
def test_parse_invalid_activation_code(code):
  with pytest.raises(ValueError):
    parse_lpa_activation_code(code)


def test_qr_validator_valid():
  valid, reason = validate_lpa_activation_code("LPA:1$rsp.truphone.com$QRF-123")
  assert valid
  assert reason == ""


def test_qr_validator_invalid():
  valid, reason = validate_lpa_activation_code("https://example.com")
  assert not valid
  assert reason
