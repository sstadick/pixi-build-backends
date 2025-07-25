use std::path::{Path, PathBuf};

use indexmap::IndexMap;
use pixi_build_backend::generated_recipe::BackendConfig;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct MojoBackendConfig {
    /// Environment Variables
    #[serde(default)]
    pub env: IndexMap<String, String>,

    /// Directory that will be created to place output artifacts.
    ///
    /// This is releative to the manifest dir.
    #[serde(default = "default_dist_dir")]
    pub dist_dir: PathBuf,

    /// Dir that can be specified for outputting pixi debug state.
    pub debug_dir: Option<PathBuf>,

    /// Binary executables to produce.
    pub bins: Option<Vec<MojoBinConfig>>,

    /// Packages to produce.
    pub pkg: Option<MojoPkgConfig>,
}

impl BackendConfig for MojoBackendConfig {
    fn debug_dir(&self) -> Option<&Path> {
        self.debug_dir.as_deref()
    }
}

impl Default for MojoBackendConfig {
    fn default() -> Self {
        Self {
            env: Default::default(),
            dist_dir: default_dist_dir(),
            debug_dir: Default::default(),
            bins: Default::default(),
            pkg: Default::default(),
        }
    }
}

fn default_dist_dir() -> PathBuf {
    PathBuf::from("target")
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct MojoBinConfig {
    pub name: String,
    pub path: String,
    #[serde(default, rename(serialize = "extra_args"))]
    pub extra_args: Option<Vec<String>>,
    #[serde(default)]
    pub env: IndexMap<String, String>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct MojoPkgConfig {
    pub name: String,
    pub path: String,
    #[serde(default, rename(serialize = "extra_args"))]
    pub extra_args: Option<Vec<String>>,
    #[serde(default)]
    pub env: IndexMap<String, String>,
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::MojoBackendConfig;

    #[test]
    fn test_ensure_deseralize_from_empty() {
        let json_data = json!({});
        serde_json::from_value::<MojoBackendConfig>(json_data).unwrap();
    }
}
