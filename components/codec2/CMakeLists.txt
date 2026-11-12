# Codec2 voice codec — ESP-IDF component wrapper
#
# Source: https://github.com/drowe67/codec2 (src/ directory)
#
# Includes only the core voice codec files needed for 1200bps mode.
# Everything modem/FreeDV/OFDM/test-related is excluded.

file(GLOB CODEC2_SRCS "src/*.c")

# Exclude FreeDV modem stack (not needed for voice codec)
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*freedv.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*ofdm.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*ldpc.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*fsk.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*fdmdv.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*cohpsk.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*/filter\\.c$")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*modem_probe.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*octave.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*mpdecode.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*gp_interleaver.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*HRA.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*phi0.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*golay23.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*varicode.*")
# Exclude standalone test programs (have main())
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*_rx\\.c$")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*_tx\\.c$")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*/c2enc\\.c$")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*/c2dec\\.c$")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*/c2sim\\.c$")
# Exclude 700C-only files (newamp1 codebooks are ~200KB of float arrays)
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*newamp.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*codebooknewamp.*")
list(FILTER CODEC2_SRCS EXCLUDE REGEX ".*codebookmel.*")

idf_component_register(
    SRCS ${CODEC2_SRCS}
    INCLUDE_DIRS "src"
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=maybe-uninitialized
    -Wno-error=unused-variable
    -Wno-error=unused-but-set-variable
    -Wno-error=sign-compare
    -Wno-unused-parameter
    -Wno-error=format
    -Wno-error=unused-function
    -Wno-error=missing-field-initializers
    -Wno-error=incompatible-pointer-types
    -DCODEC2_MODE_1200_ONLY=1
)