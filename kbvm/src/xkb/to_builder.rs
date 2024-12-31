// use hashbrown::HashMap;
// use crate::builder::Builder;
// use crate::group_type::GroupType;
// use crate::xkb::Keymap;
// use crate::xkb::keymap::KeyType;
//
// impl Keymap {
//     pub fn to_builder(&self) -> Builder {
//         let mut builder = Builder::default();
//         let mut types = HashMap::new();
//         for ty in &self.types {
//             let mut builder = GroupType::builder(ty.modifiers);
//             for mapping in &ty.mappings {
//                 builder.map_preserve(
//                     mapping.modifiers,
//                     mapping.preserved,
//                     mapping.level.to_offset(),
//                 );
//             }
//             types.insert(&**ty as *const KeyType, builder.build());
//         }
//         for key in self.keys.values() {
//             let mut builder = builder.add_key(key.key_code);
//             for (idx, group) in key.groups.iter().enumerate() {
//                 let Some(group) = group else {
//                     continue;
//                 };
//                 let ty = &types[&(&*group.key_type as *const KeyType)];
//                 let mut builder = builder.add_group(idx, ty);
//                 for (idx, layer) in group.levels.iter().enumerate() {
//                     let mut builder = builder.add_layer(idx);
//                     match layer.actions
//                 }
//             }
//         }
//         builder
//     }
// }
