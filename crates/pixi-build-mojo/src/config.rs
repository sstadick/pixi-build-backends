use std::{
    collections::HashSet,
    path::{Path, PathBuf},
};

use indexmap::IndexMap;
use miette::Error;
use pixi_build_backend::generated_recipe::BackendConfig;
use serde::{Deserialize, Serialize};
use slug::slugify;

#[derive(Debug, Default, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct MojoBackendConfig {
    /// Environment Variables
    #[serde(default)]
    pub env: IndexMap<String, String>,

    /// Directory that will be created to place output artifacts.
    ///
    /// This is releative to the manifest dir.
    pub dist_dir: Option<PathBuf>,

    /// Dir that can be specified for outputting pixi debug state.
    pub debug_dir: Option<PathBuf>,

    /// Extra input globs to include in addition to the default ones.
    #[serde(default)]
    pub extra_input_globs: Vec<String>,

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

/// Config object for a Mojo binary.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct MojoBinConfig {
    /// Name of the binary.
    ///
    /// This will default to the slugified name of the project for the first
    /// binary selected.
    pub name: Option<String>,
    /// Path to file that has the `main` method.
    ///
    /// This will deafault to looking for a `main.mojo` file in:
    /// - `<manifest_root>/main.mojo`
    /// - `<manifest_root>/<slugified_package_name>/main.mojo`
    /// - `<manifest_root>/src/main.mojo`
    pub path: Option<String>,
    /// Extra args to pass to the compiler.
    #[serde(default, rename(serialize = "extra_args"))]
    pub extra_args: Option<Vec<String>>,
}

impl MojoBinConfig {
    /// Fill in any missing info and or try to find our default options.
    ///
    /// - If None, try to find a `main.mojo` file in manfiest_root
    /// - If any, for the first one, see if name or path need to be filled in
    /// - If any, verify that there are no name collisions
    pub fn fill_defaults(
        conf: Option<&Vec<Self>>,
        manifest_root: &PathBuf,
        slug_name: &str,
    ) -> miette::Result<(Option<Vec<Self>>, bool)> {
        let main = Self::find_main(manifest_root).map(|p| p.display().to_string());

        // No configuration specified
        if conf.is_none() {
            if let Some(main) = main {
                return Ok((
                    Some(vec![Self {
                        name: Some(slug_name.to_owned()),
                        path: Some(main),
                        ..Default::default()
                    }]),
                    true,
                ));
            } else {
                return Ok((None, false));
            }
        }

        // Some configuration specified
        let mut conf = conf.unwrap().clone(); // checked above
        if conf.is_empty() {
            return Ok((None, false));
        }

        if conf[0].name.is_none() {
            conf[0].name = Some(slug_name.to_owned());
        }
        if conf[0].path.is_none() {
            if main.is_none() {
                return Err(Error::msg("Could not find main.mojo for configured binary"));
            }
            conf[0].path = main;
        }

        // Verify no name collisions and that the rest of the binaries have a name and path
        let mut names = HashSet::new();
        for (i, c) in conf.iter().enumerate() {
            if c.name.is_none() {
                return Err(Error::msg(format!(
                    "Binary configuration {} is missing a name.",
                    i
                )));
            }
            if c.path.is_none() {
                return Err(Error::msg(format!(
                    "Binary configuration {} is missing a path.",
                    c.name.as_ref().unwrap(),
                )));
            }
            if names.contains(c.name.as_deref().unwrap()) {
                return Err(Error::msg(format!(
                    "Binary name has been used twice: {}",
                    c.name.as_ref().unwrap()
                )));
            }

            names.insert(c.name.clone().unwrap());
        }

        Ok((Some(conf), false))
    }

    fn find_main(root: &PathBuf) -> Option<PathBuf> {
        let mut path = root.join("main");
        for ext in ["mojo", "🔥"] {
            path.set_extension(ext);
            if path.exists() {
                return Some(path);
            }
        }
        None
    }
}

/// Config object for a Mojo package.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct MojoPkgConfig {
    /// Name to give the mojo package (.mojopkg suffix will be added).
    ///
    /// This will default to the slugified name of the project.
    pub name: Option<String>,
    /// Path to the directory that constitutes the package.
    ///
    /// This will default to lookingo for a folder with an `__init__.mojo` in
    /// in the following order:
    /// - `<manifest_root>/<slugified_package_name>/__init__.mojo`
    /// - `<manifest_root>/src/__init__.mojo`
    pub path: Option<String>,
    /// Extra args to pass to the compiler.
    #[serde(default, rename(serialize = "extra_args"))]
    pub extra_args: Option<Vec<String>>,
}

impl MojoPkgConfig {
    /// Fill in any missing info anod or try to find our default options.
    ///
    /// - If None, try to find a `src` or `<project_name>` dir with an `__init__.mojo` file in it.
    /// - If Some, see if name or path need to be filled in.
    pub fn fill_defaults(
        conf: Option<&Self>,
        manifest_root: &PathBuf,
        slug_name: &str,
    ) -> miette::Result<(Option<Self>, bool)> {
        if let Some(conf) = conf {
            // A conf was given, make sure it has a name and path
            let mut conf = conf.clone();
            if conf.name.is_none() {
                conf.name = Some(slug_name.to_owned());
            }

            let path = Self::find_init_parent(manifest_root, slug_name);
            if conf.path.is_none() {
                if path.is_none() {
                    return Err(Error::msg(format!(
                        "Could not find valid package path for {}",
                        conf.name.unwrap()
                    )));
                }
                conf.path = path.map(|p| p.display().to_string());
            }
            Ok((Some(conf), false))
        } else {
            // No conf given check if we can find a valid package
            let path = Self::find_init_parent(manifest_root, slug_name);
            if path.is_none() {
                return Ok((None, false));
            }
            Ok((
                Some(Self {
                    name: Some(slug_name.to_owned()),
                    path: path.map(|p| p.display().to_string()),
                    ..Default::default()
                }),
                true,
            ))
        }
    }

    fn find_init_parent(root: &PathBuf, project_name: &str) -> Option<PathBuf> {
        for dir in [project_name, "src"] {
            let mut path = root.join(dir).join("__init__");
            for ext in ["mojo", "🔥"] {
                path.set_extension(ext);
                if path.exists() {
                    return Some(root.join(dir));
                }
            }
        }
        None
    }
}

/// Slugify a name for use in [`MojoPkgConfig`] and [`MojoBinconfig`].
pub fn slugify_name<S: AsRef<str>>(s: S) -> String {
    slugify(s)
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
