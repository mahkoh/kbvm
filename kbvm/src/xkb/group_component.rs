use std::fmt::{Display, Formatter};

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub(crate) enum GroupComponent {
    None,
    Base,
    Latched,
    Locked,
    #[default]
    Effective,
}

impl Display for GroupComponent {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            GroupComponent::None => "None",
            GroupComponent::Base => "Base",
            GroupComponent::Latched => "Latched",
            GroupComponent::Locked => "Locked",
            GroupComponent::Effective => "Effective",
        };
        f.write_str(s)
    }
}
