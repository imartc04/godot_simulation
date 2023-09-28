
======== Computing dependency graph ========
Graph root
    conanfile.py: /tfm/godot_simulation/conanfile.py
Requirements
    abseil/20230125.3#5431a4c609f5fb48bb8d8567e953243f - Cache
    c-ares/1.19.1#420a0b77e370f4b96bee88ef91837ccc - Cache
    grpc/1.54.3#3e0d68cd1578502c9c3a0e8366f0ab77 - Cache
    openssl/3.1.2#8879e931d726a8aad7f372e28470faa1 - Cache
    protobuf/3.21.12#8210c0b1bb46b08ff2814614ec091a9c - Cache
    re2/20230301#56bcddd1eaca2b093fd34525ae40ee9b - Cache
    zlib/1.2.13#97d5730b529b4224045fe7090592d4c1 - Cache
Build requirements
    cmake/3.27.5#aa6b0dfc03844c3ce4bb57e2dfc33058 - Cache
    ninja/1.11.1#77587f8c8318662ac8e5a7867eb4be21 - Cache
Resolved version ranges
    grpc/[~1.54]: grpc/1.54.3
    ninja/[~1.11]: ninja/1.11.1
    openssl/[>=1.1 <4]: openssl/3.1.2

======== Computing necessary packages ========
abseil/20230125.3: Checking 5 compatible configurations
abseil/20230125.3: Main binary package '308ccc1b7065dbcf54f86f902c83ab6fcd18d025' missing. Using compatible package '4bdfd36d50e15d6c87f6d89b85d456c9fd7cb8f1': compiler.cppstd=17
re2/20230301: Checking 7 compatible configurations
re2/20230301: Main binary package 'ca36276b32506a3951e4e1f0e89dd72d6fd741be' missing. Using compatible package 'a9e567f7d82e17c61e9f1945ab46f0eb37d385f0': compiler.cppstd=17
protobuf/3.21.12: Checking 9 compatible configurations
protobuf/3.21.12: Main binary package 'decd1edc47c5934b33b832d3ca565d3f6afb398a' missing. Using compatible package '564f6aaa753a6ea624f9587e846185b5f7b7b5ff': compiler.cppstd=17
grpc/1.54.3: Checking 5 compatible configurations
grpc/1.54.3: Main binary package '34bc9623db121e271b3bf2364e5cbd4ba3213316' missing. Using compatible package 'a9a68de8ea9827594556e17fc857d64968c31cd4': compiler.cppstd=17
Requirements
    abseil/20230125.3#5431a4c609f5fb48bb8d8567e953243f:4bdfd36d50e15d6c87f6d89b85d456c9fd7cb8f1#9a405ba7ecf74acd0966367f1d0244c8 - Cache
    c-ares/1.19.1#420a0b77e370f4b96bee88ef91837ccc:8141fe240b9dca158f0212d528c290b674b1079f#2e1eda64ce2e87ab3094a6aba2eef5fe - Cache
    grpc/1.54.3#3e0d68cd1578502c9c3a0e8366f0ab77:a9a68de8ea9827594556e17fc857d64968c31cd4#b29e5dc06ea0129f273e2cd20a9cd713 - Cache
    openssl/3.1.2#8879e931d726a8aad7f372e28470faa1:2b010f4baf8e367a9e2df7f714b35852aa8fa5aa#669fa67dde17e5dc1340d9942e75e376 - Cache
    protobuf/3.21.12#8210c0b1bb46b08ff2814614ec091a9c:564f6aaa753a6ea624f9587e846185b5f7b7b5ff#d001a7a2816bb22a7c1a9e94867995f0 - Cache
    re2/20230301#56bcddd1eaca2b093fd34525ae40ee9b:a9e567f7d82e17c61e9f1945ab46f0eb37d385f0#be0d01000abe52aecfed8bf3c10b7242 - Cache
    zlib/1.2.13#97d5730b529b4224045fe7090592d4c1:72c852c5f0ae27ca0b1741e5fd7c8b8be91a590a#13d813e86f3ed001aacb076316a94475 - Cache
Build requirements
    cmake/3.27.5#aa6b0dfc03844c3ce4bb57e2dfc33058:63fead0844576fc02943e16909f08fcdddd6f44b#9c2827d1c3de269612bfe59c24b71a4b - Cache
    ninja/1.11.1#77587f8c8318662ac8e5a7867eb4be21:3593751651824fb813502c69c971267624ced41a#60e6fc0f973babfbed66a66af22a4f02 - Cache

======== Basic graph information ========
:
  ref: 
  id: 0
  recipe: Consumer
  package_id: aa5448ee1c0d59f92967622ddef493028b511142
  prev: None
  rrev: None
  rrev_timestamp: None
  prev_timestamp: None
  remote: None
  binary_remote: None
  build_id: None
  binary: None
  invalid_build: False
  info_invalid: None
  name: None
  user: None
  channel: None
  url: None
  license: None
  author: None
  description: None
  homepage: None
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options: None
  options_description: None
  version: None
  topics: None
  package_type: unknown
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.cppstd: gnu14
    compiler.libcxx: libstdc++11
    compiler.version: 9
    build_type: Release
  options:
  options_definitions:
  generators: ['CMakeToolchain', 'CMakeDeps']
  python_requires: None
  system_requires:
  recipe_folder: /tfm/godot_simulation
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: conanfile.py
  dependencies:
    1:
      ref: grpc/1.54.3
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: semver_mode
      visible: True
    2:
      ref: abseil/20230125.3
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: False
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: semver_mode
      visible: True
    3:
      ref: c-ares/1.19.1
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: False
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: False
      package_id_mode: semver_mode
      visible: True
    4:
      ref: openssl/3.1.2
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: False
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: False
      package_id_mode: semver_mode
      visible: True
    6:
      ref: re2/20230301
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: False
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: False
      package_id_mode: semver_mode
      visible: True
    7:
      ref: protobuf/3.21.12
      run: True
      libs: True
      skip: False
      test: False
      force: False
      direct: False
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: semver_mode
      visible: True
    5:
      ref: zlib/1.2.13
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: False
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: semver_mode
      visible: True
    8:
      ref: cmake/3.27.5
      run: True
      libs: False
      skip: False
      test: False
      force: False
      direct: True
      build: True
      transitive_headers: None
      transitive_libs: None
      headers: False
      package_id_mode: None
      visible: False
    9:
      ref: ninja/1.11.1
      run: True
      libs: False
      skip: False
      test: False
      force: False
      direct: True
      build: True
      transitive_headers: None
      transitive_libs: None
      headers: False
      package_id_mode: None
      visible: False
  context: host
  test: False
grpc/1.54.3#3e0d68cd1578502c9c3a0e8366f0ab77:
  ref: grpc/1.54.3#3e0d68cd1578502c9c3a0e8366f0ab77
  id: 1
  recipe: Cache
  package_id: a9a68de8ea9827594556e17fc857d64968c31cd4
  prev: b29e5dc06ea0129f273e2cd20a9cd713
  rrev: 3e0d68cd1578502c9c3a0e8366f0ab77
  rrev_timestamp: 1692188941.378
  prev_timestamp: 1695809915.9336815
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: grpc
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: Apache-2.0
  author: None
  description: Google's RPC (remote procedure call) library and framework.
  homepage: https://github.com/grpc/grpc
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options:
    shared: False
    fPIC: True
    codegen: True
    csharp_ext: False
    cpp_plugin: True
    csharp_plugin: True
    node_plugin: True
    objective_c_plugin: True
    php_plugin: True
    python_plugin: True
    ruby_plugin: True
    secure: False
  options_description: None
  version: 1.54.3
  topics: ['rpc']
  package_type: static-library
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.cppstd: 17
    compiler.libcxx: libstdc++11
    compiler.version: 9
    build_type: Release
  options:
    codegen: True
    cpp_plugin: True
    csharp_ext: False
    csharp_plugin: True
    fPIC: True
    node_plugin: True
    objective_c_plugin: True
    php_plugin: True
    python_plugin: True
    ruby_plugin: True
    secure: False
    shared: False
  options_definitions:
    shared: ['True', 'False', 'ANY']
    fPIC: ['True', 'False', 'ANY']
    codegen: ['True', 'False', 'ANY']
    csharp_ext: ['True', 'False', 'ANY']
    cpp_plugin: ['True', 'False', 'ANY']
    csharp_plugin: ['True', 'False', 'ANY']
    node_plugin: ['True', 'False', 'ANY']
    objective_c_plugin: ['True', 'False', 'ANY']
    php_plugin: ['True', 'False', 'ANY']
    python_plugin: ['True', 'False', 'ANY']
    ruby_plugin: ['True', 'False', 'ANY']
    secure: ['True', 'False', 'ANY']
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/grpc622a89d1a40c3/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: grpc/1.54.3
  dependencies:
    2:
      ref: abseil/20230125.3
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: True
      transitive_libs: True
      headers: True
      package_id_mode: minor_mode
      visible: True
    3:
      ref: c-ares/1.19.1
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: minor_mode
      visible: True
    4:
      ref: openssl/3.1.2
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: minor_mode
      visible: True
    6:
      ref: re2/20230301
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: minor_mode
      visible: True
    7:
      ref: protobuf/3.21.12
      run: True
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: True
      transitive_libs: True
      headers: True
      package_id_mode: minor_mode
      visible: True
    5:
      ref: zlib/1.2.13
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: True
      transitive_libs: True
      headers: True
      package_id_mode: minor_mode
      visible: True
  context: host
  test: False
abseil/20230125.3#5431a4c609f5fb48bb8d8567e953243f:
  ref: abseil/20230125.3#5431a4c609f5fb48bb8d8567e953243f
  id: 2
  recipe: Cache
  package_id: 4bdfd36d50e15d6c87f6d89b85d456c9fd7cb8f1
  prev: 9a405ba7ecf74acd0966367f1d0244c8
  rrev: 5431a4c609f5fb48bb8d8567e953243f
  rrev_timestamp: 1683535123.972
  prev_timestamp: 1695809716.4253805
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: abseil
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: Apache-2.0
  author: None
  description: Abseil Common Libraries (C++) from Google
  homepage: https://github.com/abseil/abseil-cpp
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options:
    shared: False
    fPIC: True
  options_description: None
  version: 20230125.3
  topics: ['algorithm', 'container', 'google', 'common', 'utility']
  package_type: static-library
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.cppstd: 17
    compiler.libcxx: libstdc++11
    compiler.version: 9
    build_type: Release
  options:
    fPIC: True
    shared: False
  options_definitions:
    shared: ['True', 'False', 'ANY']
    fPIC: ['True', 'False', 'ANY']
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/abseic0d69a22a2d17/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: abseil/20230125.3
  dependencies:
  context: host
  test: False
c-ares/1.19.1#420a0b77e370f4b96bee88ef91837ccc:
  ref: c-ares/1.19.1#420a0b77e370f4b96bee88ef91837ccc
  id: 3
  recipe: Cache
  package_id: 8141fe240b9dca158f0212d528c290b674b1079f
  prev: 2e1eda64ce2e87ab3094a6aba2eef5fe
  rrev: 420a0b77e370f4b96bee88ef91837ccc
  rrev_timestamp: 1684765050.756
  prev_timestamp: 1695809722.1865082
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: c-ares
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: MIT
  author: None
  description: A C library for asynchronous DNS requests
  homepage: https://c-ares.haxx.se/
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options:
    shared: False
    fPIC: True
    tools: True
  options_description: None
  version: 1.19.1
  topics: ['dns', 'resolver', 'async']
  package_type: static-library
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.version: 9
    build_type: Release
  options:
    fPIC: True
    shared: False
    tools: True
  options_definitions:
    shared: ['True', 'False']
    fPIC: ['True', 'False']
    tools: ['True', 'False']
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/c-are89b0ecdc8f0ea/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: c-ares/1.19.1
  dependencies:
  context: host
  test: False
openssl/3.1.2#8879e931d726a8aad7f372e28470faa1:
  ref: openssl/3.1.2#8879e931d726a8aad7f372e28470faa1
  id: 4
  recipe: Cache
  package_id: 2b010f4baf8e367a9e2df7f714b35852aa8fa5aa
  prev: 669fa67dde17e5dc1340d9942e75e376
  rrev: 8879e931d726a8aad7f372e28470faa1
  rrev_timestamp: 1694631174.88
  prev_timestamp: 1695809745.8347216
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: openssl
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: Apache-2.0
  author: None
  description: A toolkit for the Transport Layer Security (TLS) and Secure Sockets Layer (SSL) protocols
  homepage: https://github.com/openssl/openssl
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options:
    shared: False
    fPIC: True
    enable_weak_ssl_ciphers: False
    386: False
    capieng_dialog: False
    enable_capieng: False
    no_aria: False
    no_asm: False
    no_async: False
    no_blake2: False
    no_bf: False
    no_camellia: False
    no_chacha: False
    no_cms: False
    no_comp: False
    no_ct: False
    no_cast: False
    no_deprecated: False
    no_des: False
    no_dgram: False
    no_dh: False
    no_dsa: False
    no_dso: False
    no_ec: False
    no_ecdh: False
    no_ecdsa: False
    no_engine: False
    no_filenames: False
    no_fips: False
    no_gost: False
    no_idea: False
    no_legacy: False
    no_md2: True
    no_md4: False
    no_mdc2: False
    no_module: False
    no_ocsp: False
    no_pinshared: False
    no_rc2: False
    no_rc4: False
    no_rc5: False
    no_rfc3779: False
    no_rmd160: False
    no_sm2: False
    no_sm3: False
    no_sm4: False
    no_srp: False
    no_srtp: False
    no_sse2: False
    no_ssl: False
    no_stdio: False
    no_seed: False
    no_sock: False
    no_ssl3: False
    no_threads: False
    no_tls1: False
    no_ts: False
    no_whirlpool: False
    no_zlib: False
    openssldir: None
  options_description: None
  version: 3.1.2
  topics: ['ssl', 'tls', 'encryption', 'security']
  package_type: static-library
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.version: 9
    build_type: Release
  options:
    386: False
    enable_weak_ssl_ciphers: False
    fPIC: True
    no_aria: False
    no_asm: False
    no_async: False
    no_bf: False
    no_blake2: False
    no_camellia: False
    no_cast: False
    no_chacha: False
    no_cms: False
    no_comp: False
    no_ct: False
    no_deprecated: False
    no_des: False
    no_dgram: False
    no_dh: False
    no_dsa: False
    no_dso: False
    no_ec: False
    no_ecdh: False
    no_ecdsa: False
    no_engine: False
    no_filenames: False
    no_fips: False
    no_gost: False
    no_idea: False
    no_legacy: False
    no_md2: True
    no_md4: False
    no_mdc2: False
    no_module: False
    no_ocsp: False
    no_pinshared: False
    no_rc2: False
    no_rc4: False
    no_rc5: False
    no_rfc3779: False
    no_rmd160: False
    no_seed: False
    no_sm2: False
    no_sm3: False
    no_sm4: False
    no_sock: False
    no_srp: False
    no_srtp: False
    no_sse2: False
    no_ssl: False
    no_ssl3: False
    no_stdio: False
    no_threads: False
    no_tls1: False
    no_ts: False
    no_whirlpool: False
    no_zlib: False
    openssldir: None
    shared: False
  options_definitions:
    shared: ['True', 'False']
    fPIC: ['True', 'False']
    enable_weak_ssl_ciphers: ['True', 'False']
    386: ['True', 'False']
    no_aria: ['True', 'False']
    no_asm: ['True', 'False']
    no_async: ['True', 'False']
    no_blake2: ['True', 'False']
    no_bf: ['True', 'False']
    no_camellia: ['True', 'False']
    no_chacha: ['True', 'False']
    no_cms: ['True', 'False']
    no_comp: ['True', 'False']
    no_ct: ['True', 'False']
    no_cast: ['True', 'False']
    no_deprecated: ['True', 'False']
    no_des: ['True', 'False']
    no_dgram: ['True', 'False']
    no_dh: ['True', 'False']
    no_dsa: ['True', 'False']
    no_dso: ['True', 'False']
    no_ec: ['True', 'False']
    no_ecdh: ['True', 'False']
    no_ecdsa: ['True', 'False']
    no_engine: ['True', 'False']
    no_filenames: ['True', 'False']
    no_fips: ['True', 'False']
    no_gost: ['True', 'False']
    no_idea: ['True', 'False']
    no_legacy: ['True', 'False']
    no_md2: ['True', 'False']
    no_md4: ['True', 'False']
    no_mdc2: ['True', 'False']
    no_module: ['True', 'False']
    no_ocsp: ['True', 'False']
    no_pinshared: ['True', 'False']
    no_rc2: ['True', 'False']
    no_rc4: ['True', 'False']
    no_rc5: ['True', 'False']
    no_rfc3779: ['True', 'False']
    no_rmd160: ['True', 'False']
    no_sm2: ['True', 'False']
    no_sm3: ['True', 'False']
    no_sm4: ['True', 'False']
    no_srp: ['True', 'False']
    no_srtp: ['True', 'False']
    no_sse2: ['True', 'False']
    no_ssl: ['True', 'False']
    no_stdio: ['True', 'False']
    no_seed: ['True', 'False']
    no_sock: ['True', 'False']
    no_ssl3: ['True', 'False']
    no_threads: ['True', 'False']
    no_tls1: ['True', 'False']
    no_ts: ['True', 'False']
    no_whirlpool: ['True', 'False']
    no_zlib: ['True', 'False']
    openssldir: [None, 'ANY']
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/opens4eb1e48c45bc5/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: openssl/3.1.2
  dependencies:
    5:
      ref: zlib/1.2.13
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: minor_mode
      visible: True
  context: host
  test: False
zlib/1.2.13#97d5730b529b4224045fe7090592d4c1:
  ref: zlib/1.2.13#97d5730b529b4224045fe7090592d4c1
  id: 5
  recipe: Cache
  package_id: 72c852c5f0ae27ca0b1741e5fd7c8b8be91a590a
  prev: 13d813e86f3ed001aacb076316a94475
  rrev: 97d5730b529b4224045fe7090592d4c1
  rrev_timestamp: 1692672717.049
  prev_timestamp: 1695809727.556452
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: zlib
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: Zlib
  author: None
  description: A Massively Spiffy Yet Delicately Unobtrusive Compression Library (Also Free, Not to Mention Unencumbered by Patents)
  homepage: https://zlib.net
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options:
    shared: False
    fPIC: True
  options_description: None
  version: 1.2.13
  topics: ['zlib', 'compression']
  package_type: static-library
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.version: 9
    build_type: Release
  options:
    fPIC: True
    shared: False
  options_definitions:
    shared: ['True', 'False']
    fPIC: ['True', 'False']
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/zlib80930ad262755/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: zlib/1.2.13
  dependencies:
  context: host
  test: False
re2/20230301#56bcddd1eaca2b093fd34525ae40ee9b:
  ref: re2/20230301#56bcddd1eaca2b093fd34525ae40ee9b
  id: 6
  recipe: Cache
  package_id: a9e567f7d82e17c61e9f1945ab46f0eb37d385f0
  prev: be0d01000abe52aecfed8bf3c10b7242
  rrev: 56bcddd1eaca2b093fd34525ae40ee9b
  rrev_timestamp: 1688369109.479
  prev_timestamp: 1695809724.9408445
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: re2
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: BSD-3-Clause
  author: None
  description: Fast, safe, thread-friendly regular expression library
  homepage: https://github.com/google/re2
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options:
    shared: False
    fPIC: True
    with_icu: False
  options_description: None
  version: 20230301
  topics: ['regex']
  package_type: static-library
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.cppstd: 17
    compiler.libcxx: libstdc++11
    compiler.version: 9
    build_type: Release
  options:
    fPIC: True
    shared: False
    with_icu: False
  options_definitions:
    shared: ['True', 'False', 'ANY']
    fPIC: ['True', 'False', 'ANY']
    with_icu: ['True', 'False', 'ANY']
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/re225b95a892bd0d/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: re2/20230301
  dependencies:
  context: host
  test: False
protobuf/3.21.12#8210c0b1bb46b08ff2814614ec091a9c:
  ref: protobuf/3.21.12#8210c0b1bb46b08ff2814614ec091a9c
  id: 7
  recipe: Cache
  package_id: 564f6aaa753a6ea624f9587e846185b5f7b7b5ff
  prev: d001a7a2816bb22a7c1a9e94867995f0
  rrev: 8210c0b1bb46b08ff2814614ec091a9c
  rrev_timestamp: 1694433328.915
  prev_timestamp: 1695809772.4923499
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: protobuf
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: BSD-3-Clause
  author: None
  description: Protocol Buffers - Google's data interchange format
  homepage: https://github.com/protocolbuffers/protobuf
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options:
    shared: False
    fPIC: True
    with_zlib: True
    with_rtti: True
    lite: False
    debug_suffix: True
  options_description: None
  version: 3.21.12
  topics: ['protocol-buffers', 'protocol-compiler', 'serialization', 'rpc', 'protocol-compiler']
  package_type: static-library
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.cppstd: 17
    compiler.libcxx: libstdc++11
    compiler.version: 9
    build_type: Release
  options:
    debug_suffix: True
    fPIC: True
    lite: False
    shared: False
    with_rtti: True
    with_zlib: True
  options_definitions:
    shared: ['True', 'False', 'ANY']
    fPIC: ['True', 'False', 'ANY']
    with_zlib: ['True', 'False', 'ANY']
    with_rtti: ['True', 'False', 'ANY']
    lite: ['True', 'False', 'ANY']
    debug_suffix: ['True', 'False', 'ANY']
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/protoac418491943b8/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: protobuf/3.21.12
  dependencies:
    5:
      ref: zlib/1.2.13
      run: False
      libs: True
      skip: False
      test: False
      force: False
      direct: True
      build: False
      transitive_headers: None
      transitive_libs: None
      headers: True
      package_id_mode: minor_mode
      visible: True
  context: host
  test: False
cmake/3.27.5#aa6b0dfc03844c3ce4bb57e2dfc33058:
  ref: cmake/3.27.5#aa6b0dfc03844c3ce4bb57e2dfc33058
  id: 8
  recipe: Cache
  package_id: 63fead0844576fc02943e16909f08fcdddd6f44b
  prev: 9c2827d1c3de269612bfe59c24b71a4b
  rrev: aa6b0dfc03844c3ce4bb57e2dfc33058
  rrev_timestamp: 1694946207.837
  prev_timestamp: 1694946561.363
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: cmake
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: BSD-3-Clause
  author: None
  description: CMake, the cross-platform, open-source build system.
  homepage: https://github.com/Kitware/CMake
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options: None
  options_description: None
  version: 3.27.5
  topics: ['build', 'installer']
  package_type: application
  settings:
    os: Linux
    arch: x86_64
  options:
  options_definitions:
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/cmake6fc2de41983d8/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: cmake/3.27.5
  dependencies:
  context: build
  test: False
ninja/1.11.1#77587f8c8318662ac8e5a7867eb4be21:
  ref: ninja/1.11.1#77587f8c8318662ac8e5a7867eb4be21
  id: 9
  recipe: Cache
  package_id: 3593751651824fb813502c69c971267624ced41a
  prev: 60e6fc0f973babfbed66a66af22a4f02
  rrev: 77587f8c8318662ac8e5a7867eb4be21
  rrev_timestamp: 1684431244.21
  prev_timestamp: 1684431632.795
  remote: None
  binary_remote: None
  build_id: None
  binary: Cache
  invalid_build: False
  info_invalid: None
  name: ninja
  user: None
  channel: None
  url: https://github.com/conan-io/conan-center-index
  license: Apache-2.0
  author: None
  description: Ninja is a small build system with a focus on speed
  homepage: https://github.com/ninja-build/ninja
  build_policy: None
  upload_policy: None
  revision_mode: hash
  provides: None
  deprecated: None
  win_bash: None
  win_bash_run: None
  default_options: None
  options_description: None
  version: 1.11.1
  topics: ['ninja', 'build']
  package_type: application
  settings:
    os: Linux
    arch: x86_64
    compiler: gcc
    compiler.cppstd: gnu14
    compiler.libcxx: libstdc++11
    compiler.version: 9
    build_type: Release
  options:
  options_definitions:
  generators: []
  python_requires: None
  system_requires:
  recipe_folder: /root/.conan2/p/ninja19c9f8e277acc/e
  source_folder: None
  build_folder: None
  generators_folder: None
  package_folder: None
  cpp_info:
    root:
      includedirs: ['include']
      srcdirs: None
      libdirs: ['lib']
      resdirs: None
      bindirs: ['bin']
      builddirs: None
      frameworkdirs: None
      system_libs: None
      frameworks: None
      libs: None
      defines: None
      cflags: None
      cxxflags: None
      sharedlinkflags: None
      exelinkflags: None
      objects: None
      sysroot: None
      requires: None
      properties: None
  conf_info:
  label: ninja/1.11.1
  dependencies:
  context: build
  test: False
