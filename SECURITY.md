# Security Policy

## Supported Versions

| Version | Supported |
| --- | --- |
| `main` | ✅ Supported |
| `v0.1.0` | ✅ Supported |

## Reporting a Vulnerability

Please do **not** disclose security vulnerabilities publicly. Report security
issues privately through a **GitHub Security Advisory** on this repository.

Include:

1. The affected file and line.
2. A description of the issue and its impact.
3. Reproduction steps, if any.
4. A suggested fix, if you have one.

## Security posture

- This is a simulation/research project; no secrets are stored in the
  repository.
- Physics backends load world geometry from in-repo definitions only — no
  arbitrary remote content.
- CI runs with least-privilege permissions and pinned action versions.
