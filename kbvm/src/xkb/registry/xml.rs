use {
    serde::{Deserialize, Deserializer},
    std::option,
};

#[derive(Deserialize)]
pub(super) struct Registry {
    #[serde(rename = "modelList")]
    #[serde(deserialize_with = "model_list")]
    models: Vec<Model>,
    #[serde(rename = "layoutList")]
    #[serde(deserialize_with = "layout_list")]
    layouts: Vec<Layout>,
    #[serde(rename = "optionList")]
    #[serde(deserialize_with = "option_list")]
    options: Vec<OptionGroup>,
}

#[derive(Deserialize, Default, Clone, Debug, Eq, PartialEq)]
#[serde(rename_all = "lowercase")]
enum Popularity {
    #[default]
    Standard,
    Exotic,
}

#[derive(Deserialize)]
struct Model {
    #[serde(rename = "configItem")]
    config_item: ConfigItem,
}

#[derive(Deserialize)]
struct Layout {
    #[serde(rename = "configItem")]
    config_item: ConfigItem,
    #[serde(rename = "variantList")]
    #[serde(default)]
    #[serde(deserialize_with = "variant_list")]
    variants: Vec<Variant>,
}

#[derive(Deserialize)]
struct Variant {
    #[serde(rename = "configItem")]
    config_item: ConfigItem,
}

#[derive(Deserialize)]
struct OptionGroup {
    #[serde(rename = "@allowMultipleSelection")]
    #[serde(default)]
    allow_multiple_selection: bool,
    #[serde(rename = "configItem")]
    config_item: ConfigItem,
    #[serde(rename = "option")]
    options: Vec<Option>,
}

#[derive(Deserialize)]
struct Option {
    #[serde(rename = "configItem")]
    config_item: ConfigItem,
}

#[derive(Deserialize)]
struct ConfigItem {
    #[serde(rename = "@popularity")]
    #[serde(default)]
    popularity: Popularity,
    name: String,
    #[serde(rename = "shortDescription")]
    #[serde(default)]
    short_description: option::Option<String>,
    #[serde(default)]
    description: option::Option<String>,
    #[serde(default)]
    vendor: option::Option<String>,
    #[serde(rename = "countryList")]
    #[serde(deserialize_with = "country_list")]
    #[serde(default)]
    countries: Vec<String>,
    #[serde(rename = "languageList")]
    #[serde(deserialize_with = "language_list")]
    #[serde(default)]
    languages: Vec<String>,
    #[serde(rename = "hwList")]
    #[serde(deserialize_with = "hw_list")]
    #[serde(default)]
    _hw_ids: Vec<String>,
}

macro_rules! nested_list {
    ($fn:ident, $ty:ty, $name:expr) => {
        fn $fn<'de, D>(deserializer: D) -> Result<Vec<$ty>, D::Error>
        where
            D: Deserializer<'de>,
        {
            #[derive(Deserialize)]
            struct List {
                #[serde(rename = $name)]
                #[serde(default)]
                elements: Vec<$ty>,
            }
            Ok(List::deserialize(deserializer)?.elements)
        }
    };
}

nested_list!(country_list, String, "iso3166Id");
nested_list!(hw_list, String, "hwId");
nested_list!(language_list, String, "iso639Id");
nested_list!(layout_list, Layout, "layout");
nested_list!(model_list, Model, "model");
nested_list!(option_list, OptionGroup, "group");
nested_list!(variant_list, Variant, "variant");

impl From<Model> for super::Model {
    fn from(value: Model) -> Self {
        super::Model {
            name: value.config_item.name,
            description: value.config_item.description,
            vendor: value.config_item.vendor,
            popularity: value.config_item.popularity.into(),
        }
    }
}

impl From<Option> for super::Opt {
    fn from(value: Option) -> Self {
        super::Opt {
            name: value.config_item.name,
            short_description: value.config_item.short_description,
            description: value.config_item.description,
            popularity: value.config_item.popularity.into(),
        }
    }
}

impl From<OptionGroup> for super::OptGroup {
    fn from(value: OptionGroup) -> Self {
        super::OptGroup {
            allow_multiple: value.allow_multiple_selection,
            name: value.config_item.name,
            description: value.config_item.description,
            popularity: value.config_item.popularity.into(),
            options: value.options.into_iter().map(|o| o.into()).collect(),
        }
    }
}

impl From<Variant> for super::Variant {
    fn from(value: Variant) -> Self {
        super::Variant {
            name: value.config_item.name,
            short_description: value.config_item.short_description,
            description: value.config_item.description,
            popularity: value.config_item.popularity.into(),
            languages: value.config_item.languages,
            countries: value.config_item.countries,
        }
    }
}

impl From<Layout> for super::Layout {
    fn from(value: Layout) -> Self {
        super::Layout {
            name: value.config_item.name,
            short_description: value.config_item.short_description,
            description: value.config_item.description,
            popularity: value.config_item.popularity.into(),
            languages: value.config_item.languages,
            countries: value.config_item.countries,
            variants: value.variants.into_iter().map(|o| o.into()).collect(),
        }
    }
}

impl From<Popularity> for super::Popularity {
    fn from(value: Popularity) -> Self {
        match value {
            Popularity::Standard => super::Popularity::Standard,
            Popularity::Exotic => super::Popularity::Exotic,
        }
    }
}

impl From<Registry> for super::Registry {
    fn from(value: Registry) -> Self {
        super::Registry {
            models: value.models.into_iter().map(|m| m.into()).collect(),
            layouts: value.layouts.into_iter().map(|l| l.into()).collect(),
            options: value.options.into_iter().map(|o| o.into()).collect(),
        }
    }
}
