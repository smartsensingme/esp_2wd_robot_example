function test_robot_serial_read_exact ()
  addpath (fullfile (fileparts (mfilename ("fullpath")), "..", "components", ...
                    "esp_2wd_robot", "tools", "octave"));
  global robot_test_chunks;
  expected = uint8 (mod ((0:511)', 256));
  % Reproduce the reported 303-byte first read, followed by USB fragments.
  robot_test_chunks = {expected(1:303), uint8([]), expected(304:367), expected(368:end)};
  actual = robot_serial_read_exact ([], 512, 1, @fragment_reader);
  assert (isequal (actual, expected));
  robot_test_chunks = {expected(1:303)};
  rejected = false;
  try
    robot_serial_read_exact ([], 512, 0.02, @fragment_reader);
  catch err
    rejected = ! isempty (strfind (err.message, "expected 512 bytes, got 303"));
  end_try_catch
  assert (rejected);
  clear global robot_test_chunks;
  fprintf ("Robot partial binary read and truncated-transfer tests passed\n");
endfunction

function chunk = fragment_reader (~, count)
  global robot_test_chunks;
  if (isempty (robot_test_chunks))
    chunk = uint8 ([]);
  else
    chunk = robot_test_chunks{1};
    robot_test_chunks(1) = [];
    assert (numel (chunk) <= count);
  endif
endfunction
