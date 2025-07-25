use std::path::{Path, PathBuf};

use indexmap::IndexMap;
use pixi_build_backend::generated_recipe::BackendConfig;
use serde::Deserialize;

#[derive(Debug, Default, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct MojoBackendConfig {
    /// Extra args for mojo invocation
    #[serde(default)]
    pub extra_args: Vec<String>,
    /// Environment Variables
    #[serde(default)]
    pub env: IndexMap<String, String>,

    pub debug_dir: Option<PathBuf>,
}

impl BackendConfig for MojoBackendConfig {
    fn debug_dir(&self) -> Option<&Path> {
        self.debug_dir.as_deref()
    }
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
