use super::*;

#[cfg(test)]
pub(super) const LEN: usize = 2449;

pub(super) const LONGEST_NAME: usize = 27;

pub(super) static NAMES: &str = "NoSymbolVoidSymbolBackSpaceTabLinefeedClearReturnPauseScroll_LockSys_ReqEscapeDeleteMulti_keySunComposeCodeinputKanji_BangouHangul_CodeinputSingleCandidateHangul_SingleCandidateMultipleCandidateZen_KohoHangul_MultipleCandidatePreviousCandidateMae_KohoHangul_PreviousCandidateKanjiMuhenkanHenkan_ModeHenkanRomajiHiraganaKatakanaHiragana_KatakanaZenkakuHankakuZenkaku_HankakuTourokuMassyoKana_LockKana_ShiftEisu_ShiftEisu_toggleHomeLeftUpRightDownPriorPage_UpSunPageUpNextPage_DownSunPageDownEndBeginSelectPrintSunPrint_ScreenExecuteInsertUndoSunUndoRedoSunAgainMenuFindSunFindCancelSunStopHelpBreakMode_switchscript_switchISO_Group_Shiftkana_switchArabic_switchGreek_switchHebrew_switchHangul_switchSunAltGraphNum_LockKP_SpaceKP_TabKP_EnterKP_F1KP_F2KP_F3KP_F4KP_HomeKP_LeftKP_UpKP_RightKP_DownKP_PriorKP_Page_UpKP_NextKP_Page_DownKP_EndKP_BeginKP_InsertKP_DeleteKP_EqualKP_MultiplyKP_AddKP_SeparatorKP_SubtractKP_DecimalKP_DivideKP_0KP_1KP_2KP_3KP_4KP_5KP_6KP_7KP_8KP_9F1F2F3F4F5F6F7F8F9F10F11L1F12L2F13L3F14L4F15L5F16L6F17L7F18L8F19L9F20L10F21R1F22R2F23R3F24R4F25R5F26R6F27R7F28R8F29R9F30R10F31R11F32R12F33R13F34R14F35R15Shift_LShift_RControl_LControl_RCaps_LockShift_LockMeta_LMeta_RAlt_LAlt_RSuper_LSuper_RHyper_LHyper_RISO_LockISO_Level2_LatchISO_Level3_ShiftISO_Level3_LatchISO_Level3_LockISO_Level5_ShiftISO_Level5_LatchISO_Level5_LockISO_Group_LatchISO_Group_LockISO_Next_GroupISO_Next_Group_LockISO_Prev_GroupISO_Prev_Group_LockISO_First_GroupISO_First_Group_LockISO_Last_GroupISO_Last_Group_LockISO_Left_TabISO_Move_Line_UpISO_Move_Line_DownISO_Partial_Line_UpISO_Partial_Line_DownISO_Partial_Space_LeftISO_Partial_Space_RightISO_Set_Margin_LeftISO_Set_Margin_RightISO_Release_Margin_LeftISO_Release_Margin_RightISO_Release_Both_MarginsISO_Fast_Cursor_LeftISO_Fast_Cursor_RightISO_Fast_Cursor_UpISO_Fast_Cursor_DownISO_Continuous_UnderlineISO_Discontinuous_UnderlineISO_EmphasizeISO_Center_ObjectISO_Enterdead_gravedead_acutedead_circumflexdead_tildedead_perispomenidead_macrondead_brevedead_abovedotdead_diaeresisdead_aboveringdead_doubleacutedead_carondead_cedilladead_ogonekdead_iotadead_voiced_sounddead_semivoiced_sounddead_belowdotdead_hookdead_horndead_strokedead_abovecommadead_psilidead_abovereversedcommadead_dasiadead_doublegravedead_belowringdead_belowmacrondead_belowcircumflexdead_belowtildedead_belowbrevedead_belowdiaeresisdead_invertedbrevedead_belowcommadead_currencydead_lowlinedead_aboveverticallinedead_belowverticallinedead_longsolidusoverlaydead_adead_Adead_edead_Edead_idead_Idead_odead_Odead_udead_Udead_small_schwadead_schwadead_capital_schwadead_SCHWAdead_greekdead_hamzaFirst_Virtual_ScreenPrev_Virtual_ScreenNext_Virtual_ScreenLast_Virtual_ScreenTerminate_ServerAccessX_EnableAccessX_Feedback_EnableRepeatKeys_EnableSlowKeys_EnableBounceKeys_EnableStickyKeys_EnableMouseKeys_EnableMouseKeys_Accel_EnableOverlay1_EnableOverlay2_EnableAudibleBell_EnablePointer_LeftPointer_RightPointer_UpPointer_DownPointer_UpLeftPointer_UpRightPointer_DownLeftPointer_DownRightPointer_Button_DfltPointer_Button1Pointer_Button2Pointer_Button3Pointer_Button4Pointer_Button5Pointer_DblClick_DfltPointer_DblClick1Pointer_DblClick2Pointer_DblClick3Pointer_DblClick4Pointer_DblClick5Pointer_Drag_DfltPointer_Drag1Pointer_Drag2Pointer_Drag3Pointer_Drag4Pointer_Drag5Pointer_EnableKeysPointer_AcceleratePointer_DfltBtnNextPointer_DfltBtnPrevchChCHc_hC_hC_H3270_Duplicate3270_FieldMark3270_Right23270_Left23270_BackTab3270_EraseEOF3270_EraseInput3270_Reset3270_Quit3270_PA13270_PA23270_PA33270_Test3270_Attn3270_CursorBlink3270_AltCursor3270_KeyClick3270_Jump3270_Ident3270_Rule3270_Copy3270_Play3270_Setup3270_Record3270_ChangeScreen3270_DeleteWord3270_ExSelect3270_CursorSelect3270_PrintScreen3270_Enterspaceexclamquotedblnumbersigndollarpercentampersandapostrophequoterightparenleftparenrightasteriskpluscommaminusperiodslash0123456789colonsemicolonlessequalgreaterquestionatABCDEFGHIJKLMNOPQRSTUVWXYZbracketleftbackslashbracketrightasciicircumunderscoregravequoteleftabcdefghijklmnopqrstuvwxyzbraceleftbarbracerightasciitildenobreakspaceexclamdowncentsterlingcurrencyyenbrokenbarsectiondiaeresiscopyrightordfeminineguillemotleftguillemetleftnotsignhyphenregisteredmacrondegreeplusminustwosuperiorthreesuperioracutemuparagraphperiodcenteredcedillaonesuperiormasculineordmasculineguillemotrightguillemetrightonequarteronehalfthreequartersquestiondownAgraveAacuteAcircumflexAtildeAdiaeresisAringAECcedillaEgraveEacuteEcircumflexEdiaeresisIgraveIacuteIcircumflexIdiaeresisETHEthNtildeOgraveOacuteOcircumflexOtildeOdiaeresismultiplyOslashOobliqueUgraveUacuteUcircumflexUdiaeresisYacuteTHORNThornssharpagraveaacuteacircumflexatildeadiaeresisaringaeccedillaegraveeacuteecircumflexediaeresisigraveiacuteicircumflexidiaeresisethntildeograveoacuteocircumflexotildeodiaeresisdivisionoslashoobliqueugraveuacuteucircumflexudiaeresisyacutethornydiaeresisAogonekbreveLstrokeLcaronSacuteScaronScedillaTcaronZacuteZcaronZabovedotaogonekogoneklstrokelcaronsacutecaronscaronscedillatcaronzacutedoubleacutezcaronzabovedotRacuteAbreveLacuteCacuteCcaronEogonekEcaronDcaronDstrokeNacuteNcaronOdoubleacuteRcaronUringUdoubleacuteTcedillaracuteabrevelacutecacuteccaroneogonekecarondcarondstrokenacutencaronodoubleacutercaronuringudoubleacutetcedillaabovedotHstrokeHcircumflexIabovedotGbreveJcircumflexhstrokehcircumflexidotlessgbrevejcircumflexCabovedotCcircumflexGabovedotGcircumflexUbreveScircumflexcabovedotccircumflexgabovedotgcircumflexubrevescircumflexkrakappaRcedillaItildeLcedillaEmacronGcedillaTslashrcedillaitildelcedillaemacrongcedillatslashENGengAmacronIogonekEabovedotImacronNcedillaOmacronKcedillaUogonekUtildeUmacronamacroniogonekeabovedotimacronncedillaomacronkcedillauogonekutildeumacronWcircumflexwcircumflexYcircumflexycircumflexBabovedotbabovedotDabovedotdabovedotFabovedotfabovedotMabovedotmabovedotPabovedotpabovedotSabovedotsabovedotTabovedottabovedotWgravewgraveWacutewacuteWdiaeresiswdiaeresisYgraveygraveOEoeYdiaeresisoverlinekana_fullstopkana_openingbracketkana_closingbracketkana_commakana_conjunctivekana_middledotkana_WOkana_akana_ikana_ukana_ekana_okana_yakana_yukana_yokana_tsukana_tuprolongedsoundkana_Akana_Ikana_Ukana_Ekana_Okana_KAkana_KIkana_KUkana_KEkana_KOkana_SAkana_SHIkana_SUkana_SEkana_SOkana_TAkana_CHIkana_TIkana_TSUkana_TUkana_TEkana_TOkana_NAkana_NIkana_NUkana_NEkana_NOkana_HAkana_HIkana_FUkana_HUkana_HEkana_HOkana_MAkana_MIkana_MUkana_MEkana_MOkana_YAkana_YUkana_YOkana_RAkana_RIkana_RUkana_REkana_ROkana_WAkana_NvoicedsoundsemivoicedsoundFarsi_0Farsi_1Farsi_2Farsi_3Farsi_4Farsi_5Farsi_6Farsi_7Farsi_8Farsi_9Arabic_percentArabic_superscript_alefArabic_ttehArabic_pehArabic_tchehArabic_ddalArabic_rrehArabic_commaArabic_fullstopArabic_0Arabic_1Arabic_2Arabic_3Arabic_4Arabic_5Arabic_6Arabic_7Arabic_8Arabic_9Arabic_semicolonArabic_question_markArabic_hamzaArabic_maddaonalefArabic_hamzaonalefArabic_hamzaonwawArabic_hamzaunderalefArabic_hamzaonyehArabic_alefArabic_behArabic_tehmarbutaArabic_tehArabic_thehArabic_jeemArabic_hahArabic_khahArabic_dalArabic_thalArabic_raArabic_zainArabic_seenArabic_sheenArabic_sadArabic_dadArabic_tahArabic_zahArabic_ainArabic_ghainArabic_tatweelArabic_fehArabic_qafArabic_kafArabic_lamArabic_meemArabic_noonArabic_haArabic_hehArabic_wawArabic_alefmaksuraArabic_yehArabic_fathatanArabic_dammatanArabic_kasratanArabic_fathaArabic_dammaArabic_kasraArabic_shaddaArabic_sukunArabic_madda_aboveArabic_hamza_aboveArabic_hamza_belowArabic_jehArabic_vehArabic_kehehArabic_gafArabic_noon_ghunnaArabic_heh_doachashmeeFarsi_yehArabic_farsi_yehArabic_yeh_bareeArabic_heh_goalCyrillic_GHE_barCyrillic_ghe_barCyrillic_ZHE_descenderCyrillic_zhe_descenderCyrillic_KA_descenderCyrillic_ka_descenderCyrillic_KA_vertstrokeCyrillic_ka_vertstrokeCyrillic_EN_descenderCyrillic_en_descenderCyrillic_U_straightCyrillic_u_straightCyrillic_U_straight_barCyrillic_u_straight_barCyrillic_HA_descenderCyrillic_ha_descenderCyrillic_CHE_descenderCyrillic_che_descenderCyrillic_CHE_vertstrokeCyrillic_che_vertstrokeCyrillic_SHHACyrillic_shhaCyrillic_SCHWACyrillic_schwaCyrillic_I_macronCyrillic_i_macronCyrillic_O_barCyrillic_o_barCyrillic_U_macronCyrillic_u_macronSerbian_djeMacedonia_gjeCyrillic_ioUkrainian_ieUkranian_jeMacedonia_dseUkrainian_iUkranian_iUkrainian_yiUkranian_yiCyrillic_jeSerbian_jeCyrillic_ljeSerbian_ljeCyrillic_njeSerbian_njeSerbian_tsheMacedonia_kjeUkrainian_ghe_with_upturnByelorussian_shortuCyrillic_dzheSerbian_dzenumerosignSerbian_DJEMacedonia_GJECyrillic_IOUkrainian_IEUkranian_JEMacedonia_DSEUkrainian_IUkranian_IUkrainian_YIUkranian_YICyrillic_JESerbian_JECyrillic_LJESerbian_LJECyrillic_NJESerbian_NJESerbian_TSHEMacedonia_KJEUkrainian_GHE_WITH_UPTURNByelorussian_SHORTUCyrillic_DZHESerbian_DZECyrillic_yuCyrillic_aCyrillic_beCyrillic_tseCyrillic_deCyrillic_ieCyrillic_efCyrillic_gheCyrillic_haCyrillic_iCyrillic_shortiCyrillic_kaCyrillic_elCyrillic_emCyrillic_enCyrillic_oCyrillic_peCyrillic_yaCyrillic_erCyrillic_esCyrillic_teCyrillic_uCyrillic_zheCyrillic_veCyrillic_softsignCyrillic_yeruCyrillic_zeCyrillic_shaCyrillic_eCyrillic_shchaCyrillic_cheCyrillic_hardsignCyrillic_YUCyrillic_ACyrillic_BECyrillic_TSECyrillic_DECyrillic_IECyrillic_EFCyrillic_GHECyrillic_HACyrillic_ICyrillic_SHORTICyrillic_KACyrillic_ELCyrillic_EMCyrillic_ENCyrillic_OCyrillic_PECyrillic_YACyrillic_ERCyrillic_ESCyrillic_TECyrillic_UCyrillic_ZHECyrillic_VECyrillic_SOFTSIGNCyrillic_YERUCyrillic_ZECyrillic_SHACyrillic_ECyrillic_SHCHACyrillic_CHECyrillic_HARDSIGNGreek_ALPHAaccentGreek_EPSILONaccentGreek_ETAaccentGreek_IOTAaccentGreek_IOTAdieresisGreek_IOTAdiaeresisGreek_OMICRONaccentGreek_UPSILONaccentGreek_UPSILONdieresisGreek_OMEGAaccentGreek_accentdieresisGreek_horizbarGreek_alphaaccentGreek_epsilonaccentGreek_etaaccentGreek_iotaaccentGreek_iotadieresisGreek_iotaaccentdieresisGreek_omicronaccentGreek_upsilonaccentGreek_upsilondieresisGreek_upsilonaccentdieresisGreek_omegaaccentGreek_ALPHAGreek_BETAGreek_GAMMAGreek_DELTAGreek_EPSILONGreek_ZETAGreek_ETAGreek_THETAGreek_IOTAGreek_KAPPAGreek_LAMDAGreek_LAMBDAGreek_MUGreek_NUGreek_XIGreek_OMICRONGreek_PIGreek_RHOGreek_SIGMAGreek_TAUGreek_UPSILONGreek_PHIGreek_CHIGreek_PSIGreek_OMEGAGreek_alphaGreek_betaGreek_gammaGreek_deltaGreek_epsilonGreek_zetaGreek_etaGreek_thetaGreek_iotaGreek_kappaGreek_lamdaGreek_lambdaGreek_muGreek_nuGreek_xiGreek_omicronGreek_piGreek_rhoGreek_sigmaGreek_finalsmallsigmaGreek_tauGreek_upsilonGreek_phiGreek_chiGreek_psiGreek_omegaleftradicaltopleftradicalhorizconnectortopintegralbotintegralvertconnectortopleftsqbracketbotleftsqbrackettoprightsqbracketbotrightsqbrackettopleftparensbotleftparenstoprightparensbotrightparensleftmiddlecurlybracerightmiddlecurlybracetopleftsummationbotleftsummationtopvertsummationconnectorbotvertsummationconnectortoprightsummationbotrightsummationrightmiddlesummationlessthanequalnotequalgreaterthanequalintegralthereforevariationinfinitynablaapproximatesimilarequalifonlyifimpliesidenticalradicalincludedinincludesintersectionunionlogicalandlogicalorpartialderivativefunctionleftarrowuparrowrightarrowdownarrowblanksoliddiamondcheckerboardhtffcrlfnlvtlowrightcorneruprightcornerupleftcornerlowleftcornercrossinglineshorizlinescan1horizlinescan3horizlinescan5horizlinescan7horizlinescan9lefttrighttbotttoptvertbaremspaceenspaceem3spaceem4spacedigitspacepunctspacethinspacehairspaceemdashendashsignifblankellipsisdoubbaselinedotonethirdtwothirdsonefifthtwofifthsthreefifthsfourfifthsonesixthfivesixthscareoffigdashleftanglebracketdecimalpointrightanglebracketmarkeroneeighththreeeighthsfiveeighthsseveneighthstrademarksignaturemarktrademarkincircleleftopentrianglerightopentriangleemopencircleemopenrectangleleftsinglequotemarkrightsinglequotemarkleftdoublequotemarkrightdoublequotemarkprescriptionpermilleminutessecondslatincrosshexagramfilledrectbulletfilledlefttribulletfilledrighttribulletemfilledcircleemfilledrectenopencircbulletenopensquarebulletopenrectbulletopentribulletupopentribulletdownopenstarenfilledcircbulletenfilledsqbulletfilledtribulletupfilledtribulletdownleftpointerrightpointerclubdiamondheartmaltesecrossdaggerdoubledaggercheckmarkballotcrossmusicalsharpmusicalflatmalesymbolfemalesymboltelephonetelephonerecorderphonographcopyrightcaretsinglelowquotemarkdoublelowquotemarkcursorleftcaretrightcaretdowncaretupcaretoverbardowntackupshoedownstileunderbarjotquaduptackcircleupstiledownshoerightshoeleftshoelefttackrighttackhebrew_doublelowlinehebrew_alephhebrew_bethebrew_bethhebrew_gimelhebrew_gimmelhebrew_dalethebrew_dalethhebrew_hehebrew_wawhebrew_zainhebrew_zayinhebrew_chethebrew_hethebrew_tethebrew_tethhebrew_yodhebrew_finalkaphhebrew_kaphhebrew_lamedhebrew_finalmemhebrew_memhebrew_finalnunhebrew_nunhebrew_samechhebrew_samekhhebrew_ayinhebrew_finalpehebrew_pehebrew_finalzadehebrew_finalzadihebrew_zadehebrew_zadihebrew_qophhebrew_kufhebrew_reshhebrew_shinhebrew_tawhebrew_tafThai_kokaiThai_khokhaiThai_khokhuatThai_khokhwaiThai_khokhonThai_khorakhangThai_ngonguThai_chochanThai_chochingThai_chochangThai_sosoThai_chochoeThai_yoyingThai_dochadaThai_topatakThai_thothanThai_thonangmonthoThai_thophuthaoThai_nonenThai_dodekThai_totaoThai_thothungThai_thothahanThai_thothongThai_nonuThai_bobaimaiThai_poplaThai_phophungThai_fofaThai_phophanThai_fofanThai_phosamphaoThai_momaThai_yoyakThai_roruaThai_ruThai_lolingThai_luThai_wowaenThai_sosalaThai_sorusiThai_sosuaThai_hohipThai_lochulaThai_oangThai_honokhukThai_paiyannoiThai_saraaThai_maihanakatThai_saraaaThai_saraamThai_saraiThai_saraiiThai_saraueThai_saraueeThai_sarauThai_sarauuThai_phinthuThai_maihanakat_maithoThai_bahtThai_saraeThai_saraaeThai_saraoThai_saraaimaimuanThai_saraaimaimalaiThai_lakkhangyaoThai_maiyamokThai_maitaikhuThai_maiekThai_maithoThai_maitriThai_maichattawaThai_thanthakhatThai_nikhahitThai_leksunThai_leknungThai_leksongThai_leksamThai_leksiThai_lekhaThai_lekhokThai_lekchetThai_lekpaetThai_lekkaoHangulHangul_StartHangul_EndHangul_HanjaHangul_JamoHangul_RomajaHangul_JeonjaHangul_BanjaHangul_PreHanjaHangul_PostHanjaHangul_SpecialHangul_KiyeogHangul_SsangKiyeogHangul_KiyeogSiosHangul_NieunHangul_NieunJieujHangul_NieunHieuhHangul_DikeudHangul_SsangDikeudHangul_RieulHangul_RieulKiyeogHangul_RieulMieumHangul_RieulPieubHangul_RieulSiosHangul_RieulTieutHangul_RieulPhieufHangul_RieulHieuhHangul_MieumHangul_PieubHangul_SsangPieubHangul_PieubSiosHangul_SiosHangul_SsangSiosHangul_IeungHangul_JieujHangul_SsangJieujHangul_CieucHangul_KhieuqHangul_TieutHangul_PhieufHangul_HieuhHangul_AHangul_AEHangul_YAHangul_YAEHangul_EOHangul_EHangul_YEOHangul_YEHangul_OHangul_WAHangul_WAEHangul_OEHangul_YOHangul_UHangul_WEOHangul_WEHangul_WIHangul_YUHangul_EUHangul_YIHangul_IHangul_J_KiyeogHangul_J_SsangKiyeogHangul_J_KiyeogSiosHangul_J_NieunHangul_J_NieunJieujHangul_J_NieunHieuhHangul_J_DikeudHangul_J_RieulHangul_J_RieulKiyeogHangul_J_RieulMieumHangul_J_RieulPieubHangul_J_RieulSiosHangul_J_RieulTieutHangul_J_RieulPhieufHangul_J_RieulHieuhHangul_J_MieumHangul_J_PieubHangul_J_PieubSiosHangul_J_SiosHangul_J_SsangSiosHangul_J_IeungHangul_J_JieujHangul_J_CieucHangul_J_KhieuqHangul_J_TieutHangul_J_PhieufHangul_J_HieuhHangul_RieulYeorinHieuhHangul_SunkyeongeumMieumHangul_SunkyeongeumPieubHangul_PanSiosHangul_KkogjiDalrinIeungHangul_SunkyeongeumPhieufHangul_YeorinHieuhHangul_AraeAHangul_AraeAEHangul_J_PanSiosHangul_J_KkogjiDalrinIeungHangul_J_YeorinHieuhKorean_WonArmenian_ligature_ewArmenian_full_stopArmenian_verjaketArmenian_separation_markArmenian_butArmenian_hyphenArmenian_yentamnaArmenian_exclamArmenian_amanakArmenian_accentArmenian_sheshtArmenian_questionArmenian_paruykArmenian_AYBArmenian_aybArmenian_BENArmenian_benArmenian_GIMArmenian_gimArmenian_DAArmenian_daArmenian_YECHArmenian_yechArmenian_ZAArmenian_zaArmenian_EArmenian_eArmenian_ATArmenian_atArmenian_TOArmenian_toArmenian_ZHEArmenian_zheArmenian_INIArmenian_iniArmenian_LYUNArmenian_lyunArmenian_KHEArmenian_kheArmenian_TSAArmenian_tsaArmenian_KENArmenian_kenArmenian_HOArmenian_hoArmenian_DZAArmenian_dzaArmenian_GHATArmenian_ghatArmenian_TCHEArmenian_tcheArmenian_MENArmenian_menArmenian_HIArmenian_hiArmenian_NUArmenian_nuArmenian_SHAArmenian_shaArmenian_VOArmenian_voArmenian_CHAArmenian_chaArmenian_PEArmenian_peArmenian_JEArmenian_jeArmenian_RAArmenian_raArmenian_SEArmenian_seArmenian_VEVArmenian_vevArmenian_TYUNArmenian_tyunArmenian_REArmenian_reArmenian_TSOArmenian_tsoArmenian_VYUNArmenian_vyunArmenian_PYURArmenian_pyurArmenian_KEArmenian_keArmenian_OArmenian_oArmenian_FEArmenian_feArmenian_apostropheGeorgian_anGeorgian_banGeorgian_ganGeorgian_donGeorgian_enGeorgian_vinGeorgian_zenGeorgian_tanGeorgian_inGeorgian_kanGeorgian_lasGeorgian_manGeorgian_narGeorgian_onGeorgian_parGeorgian_zharGeorgian_raeGeorgian_sanGeorgian_tarGeorgian_unGeorgian_pharGeorgian_kharGeorgian_ghanGeorgian_qarGeorgian_shinGeorgian_chinGeorgian_canGeorgian_jilGeorgian_cilGeorgian_charGeorgian_xanGeorgian_jhanGeorgian_haeGeorgian_heGeorgian_hieGeorgian_weGeorgian_harGeorgian_hoeGeorgian_fiXabovedotIbreveZstrokeGcaronOcaronObarredxabovedotibrevezstrokegcaronocaronobarredSCHWAschwaEZHezhLbelowdotlbelowdotAbelowdotabelowdotAhookahookAcircumflexacuteacircumflexacuteAcircumflexgraveacircumflexgraveAcircumflexhookacircumflexhookAcircumflextildeacircumflextildeAcircumflexbelowdotacircumflexbelowdotAbreveacuteabreveacuteAbrevegraveabrevegraveAbrevehookabrevehookAbrevetildeabrevetildeAbrevebelowdotabrevebelowdotEbelowdotebelowdotEhookehookEtildeetildeEcircumflexacuteecircumflexacuteEcircumflexgraveecircumflexgraveEcircumflexhookecircumflexhookEcircumflextildeecircumflextildeEcircumflexbelowdotecircumflexbelowdotIhookihookIbelowdotibelowdotObelowdotobelowdotOhookohookOcircumflexacuteocircumflexacuteOcircumflexgraveocircumflexgraveOcircumflexhookocircumflexhookOcircumflextildeocircumflextildeOcircumflexbelowdotocircumflexbelowdotOhornacuteohornacuteOhorngraveohorngraveOhornhookohornhookOhorntildeohorntildeOhornbelowdotohornbelowdotUbelowdotubelowdotUhookuhookUhornacuteuhornacuteUhorngraveuhorngraveUhornhookuhornhookUhorntildeuhorntildeUhornbelowdotuhornbelowdotYbelowdotybelowdotYhookyhookYtildeytildeOhornohornUhornuhorncombining_tildecombining_gravecombining_acutecombining_hookcombining_belowdotEcuSignColonSignCruzeiroSignFFrancSignLiraSignMillSignNairaSignPesetaSignRupeeSignWonSignNewSheqelSignDongSignEuroSignzerosuperiorfoursuperiorfivesuperiorsixsuperiorsevensuperioreightsuperiorninesuperiorzerosubscriptonesubscripttwosubscriptthreesubscriptfoursubscriptfivesubscriptsixsubscriptsevensubscripteightsubscriptninesubscriptpartdifferentialemptysetelementofnotelementofcontainsassquarerootcuberootfourthrootdintegraltintegralbecauseapproxeqnotapproxeqnotidenticalstricteqbraille_dot_1braille_dot_2braille_dot_3braille_dot_4braille_dot_5braille_dot_6braille_dot_7braille_dot_8braille_dot_9braille_dot_10braille_blankbraille_dots_1braille_dots_2braille_dots_12braille_dots_3braille_dots_13braille_dots_23braille_dots_123braille_dots_4braille_dots_14braille_dots_24braille_dots_124braille_dots_34braille_dots_134braille_dots_234braille_dots_1234braille_dots_5braille_dots_15braille_dots_25braille_dots_125braille_dots_35braille_dots_135braille_dots_235braille_dots_1235braille_dots_45braille_dots_145braille_dots_245braille_dots_1245braille_dots_345braille_dots_1345braille_dots_2345braille_dots_12345braille_dots_6braille_dots_16braille_dots_26braille_dots_126braille_dots_36braille_dots_136braille_dots_236braille_dots_1236braille_dots_46braille_dots_146braille_dots_246braille_dots_1246braille_dots_346braille_dots_1346braille_dots_2346braille_dots_12346braille_dots_56braille_dots_156braille_dots_256braille_dots_1256braille_dots_356braille_dots_1356braille_dots_2356braille_dots_12356braille_dots_456braille_dots_1456braille_dots_2456braille_dots_12456braille_dots_3456braille_dots_13456braille_dots_23456braille_dots_123456braille_dots_7braille_dots_17braille_dots_27braille_dots_127braille_dots_37braille_dots_137braille_dots_237braille_dots_1237braille_dots_47braille_dots_147braille_dots_247braille_dots_1247braille_dots_347braille_dots_1347braille_dots_2347braille_dots_12347braille_dots_57braille_dots_157braille_dots_257braille_dots_1257braille_dots_357braille_dots_1357braille_dots_2357braille_dots_12357braille_dots_457braille_dots_1457braille_dots_2457braille_dots_12457braille_dots_3457braille_dots_13457braille_dots_23457braille_dots_123457braille_dots_67braille_dots_167braille_dots_267braille_dots_1267braille_dots_367braille_dots_1367braille_dots_2367braille_dots_12367braille_dots_467braille_dots_1467braille_dots_2467braille_dots_12467braille_dots_3467braille_dots_13467braille_dots_23467braille_dots_123467braille_dots_567braille_dots_1567braille_dots_2567braille_dots_12567braille_dots_3567braille_dots_13567braille_dots_23567braille_dots_123567braille_dots_4567braille_dots_14567braille_dots_24567braille_dots_124567braille_dots_34567braille_dots_134567braille_dots_234567braille_dots_1234567braille_dots_8braille_dots_18braille_dots_28braille_dots_128braille_dots_38braille_dots_138braille_dots_238braille_dots_1238braille_dots_48braille_dots_148braille_dots_248braille_dots_1248braille_dots_348braille_dots_1348braille_dots_2348braille_dots_12348braille_dots_58braille_dots_158braille_dots_258braille_dots_1258braille_dots_358braille_dots_1358braille_dots_2358braille_dots_12358braille_dots_458braille_dots_1458braille_dots_2458braille_dots_12458braille_dots_3458braille_dots_13458braille_dots_23458braille_dots_123458braille_dots_68braille_dots_168braille_dots_268braille_dots_1268braille_dots_368braille_dots_1368braille_dots_2368braille_dots_12368braille_dots_468braille_dots_1468braille_dots_2468braille_dots_12468braille_dots_3468braille_dots_13468braille_dots_23468braille_dots_123468braille_dots_568braille_dots_1568braille_dots_2568braille_dots_12568braille_dots_3568braille_dots_13568braille_dots_23568braille_dots_123568braille_dots_4568braille_dots_14568braille_dots_24568braille_dots_124568braille_dots_34568braille_dots_134568braille_dots_234568braille_dots_1234568braille_dots_78braille_dots_178braille_dots_278braille_dots_1278braille_dots_378braille_dots_1378braille_dots_2378braille_dots_12378braille_dots_478braille_dots_1478braille_dots_2478braille_dots_12478braille_dots_3478braille_dots_13478braille_dots_23478braille_dots_123478braille_dots_578braille_dots_1578braille_dots_2578braille_dots_12578braille_dots_3578braille_dots_13578braille_dots_23578braille_dots_123578braille_dots_4578braille_dots_14578braille_dots_24578braille_dots_124578braille_dots_34578braille_dots_134578braille_dots_234578braille_dots_1234578braille_dots_678braille_dots_1678braille_dots_2678braille_dots_12678braille_dots_3678braille_dots_13678braille_dots_23678braille_dots_123678braille_dots_4678braille_dots_14678braille_dots_24678braille_dots_124678braille_dots_34678braille_dots_134678braille_dots_234678braille_dots_1234678braille_dots_5678braille_dots_15678braille_dots_25678braille_dots_125678braille_dots_35678braille_dots_135678braille_dots_235678braille_dots_1235678braille_dots_45678braille_dots_145678braille_dots_245678braille_dots_1245678braille_dots_345678braille_dots_1345678braille_dots_2345678braille_dots_12345678Sinh_ngSinh_h2Sinh_aSinh_aaSinh_aeSinh_aeeSinh_iSinh_iiSinh_uSinh_uuSinh_riSinh_riiSinh_luSinh_luuSinh_eSinh_eeSinh_aiSinh_oSinh_ooSinh_auSinh_kaSinh_khaSinh_gaSinh_ghaSinh_ng2Sinh_ngaSinh_caSinh_chaSinh_jaSinh_jhaSinh_nyaSinh_jnyaSinh_njaSinh_ttaSinh_tthaSinh_ddaSinh_ddhaSinh_nnaSinh_nddaSinh_thaSinh_thhaSinh_dhaSinh_dhhaSinh_naSinh_ndhaSinh_paSinh_phaSinh_baSinh_bhaSinh_maSinh_mbaSinh_yaSinh_raSinh_laSinh_vaSinh_shaSinh_sshaSinh_saSinh_haSinh_llaSinh_faSinh_alSinh_aa2Sinh_ae2Sinh_aee2Sinh_i2Sinh_ii2Sinh_u2Sinh_uu2Sinh_ru2Sinh_e2Sinh_ee2Sinh_ai2Sinh_o2Sinh_oo2Sinh_au2Sinh_lu2Sinh_ruu2Sinh_luu2Sinh_kunddaliyaXF86ModeLockXF86MonBrightnessUpXF86MonBrightnessDownXF86KbdLightOnOffXF86KbdBrightnessUpXF86KbdBrightnessDownXF86MonBrightnessCycleXF86StandbyXF86AudioLowerVolumeXF86AudioMuteXF86AudioRaiseVolumeXF86AudioPlayXF86AudioStopXF86AudioPrevXF86AudioNextXF86HomePageXF86MailXF86StartXF86SearchXF86AudioRecordXF86CalculatorXF86MemoXF86ToDoListXF86CalendarXF86PowerDownXF86ContrastAdjustXF86RockerUpXF86RockerDownXF86RockerEnterXF86BackXF86ForwardXF86StopXF86RefreshXF86PowerOffXF86WakeUpXF86EjectXF86ScreenSaverXF86WWWXF86SleepXF86FavoritesXF86AudioPauseXF86AudioMediaXF86MyComputerXF86VendorHomeXF86LightBulbXF86ShopXF86HistoryXF86OpenURLXF86AddFavoriteXF86HotLinksXF86BrightnessAdjustXF86FinanceXF86CommunityXF86AudioRewindXF86BackForwardXF86Launch0XF86Launch1XF86Launch2XF86Launch3XF86Launch4XF86Launch5XF86Launch6XF86Launch7XF86Launch8XF86Launch9XF86LaunchAXF86LaunchBXF86LaunchCXF86LaunchDXF86LaunchEXF86LaunchFXF86ApplicationLeftXF86ApplicationRightXF86BookXF86CDXF86CalculaterXF86ClearXF86CloseXF86CopyXF86CutXF86DisplayXF86DOSXF86DocumentsXF86ExcelXF86ExplorerXF86GameXF86GoXF86iTouchXF86LogOffXF86MarketXF86MeetingXF86MenuKBXF86MenuPBXF86MySitesXF86NewXF86NewsXF86OfficeHomeXF86OpenXF86OptionXF86PasteXF86PhoneXF86QXF86ReplyXF86ReloadXF86RotateWindowsXF86RotationPBXF86RotationKBXF86SaveXF86ScrollUpXF86ScrollDownXF86ScrollClickXF86SendXF86SpellXF86SplitScreenXF86SupportXF86TaskPaneXF86TerminalXF86ToolsXF86TravelXF86UserPBXF86User1KBXF86User2KBXF86VideoXF86WheelButtonXF86WordXF86XferXF86ZoomInXF86ZoomOutXF86AwayXF86MessengerXF86WebCamXF86MailForwardXF86PicturesXF86MusicXF86BatteryXF86BluetoothXF86WLANXF86UWBXF86AudioForwardXF86AudioRepeatXF86AudioRandomPlayXF86SubtitleXF86AudioCycleTrackXF86CycleAngleXF86FrameBackXF86FrameForwardXF86TimeXF86SelectXF86ViewXF86TopMenuXF86RedXF86GreenXF86YellowXF86BlueXF86SuspendXF86HibernateXF86TouchpadToggleXF86TouchpadOnXF86TouchpadOffXF86AudioMicMuteXF86KeyboardXF86WWANXF86RFKillXF86AudioPresetXF86RotationLockToggleXF86FullScreenXF86Switch_VT_1XF86Switch_VT_2XF86Switch_VT_3XF86Switch_VT_4XF86Switch_VT_5XF86Switch_VT_6XF86Switch_VT_7XF86Switch_VT_8XF86Switch_VT_9XF86Switch_VT_10XF86Switch_VT_11XF86Switch_VT_12XF86UngrabXF86ClearGrabXF86Next_VModeXF86Prev_VModeXF86LogWindowTreeXF86LogGrabInfoXF86BrightnessAutoXF86DisplayOffXF86InfoXF86AspectRatioXF86DVDXF86AudioXF86ChannelUpXF86ChannelDownXF86BreakXF86VideoPhoneXF86ZoomResetXF86EditorXF86GraphicsEditorXF86PresentationXF86DatabaseXF86VoicemailXF86AddressbookXF86DisplayToggleXF86SpellCheckXF86ContextMenuXF86MediaRepeatXF8610ChannelsUpXF8610ChannelsDownXF86ImagesXF86NotificationCenterXF86PickupPhoneXF86HangupPhoneXF86FnXF86Fn_EscXF86FnRightShiftXF86Numeric0XF86Numeric1XF86Numeric2XF86Numeric3XF86Numeric4XF86Numeric5XF86Numeric6XF86Numeric7XF86Numeric8XF86Numeric9XF86NumericStarXF86NumericPoundXF86NumericAXF86NumericBXF86NumericCXF86NumericDXF86CameraFocusXF86WPSButtonXF86CameraZoomInXF86CameraZoomOutXF86CameraUpXF86CameraDownXF86CameraLeftXF86CameraRightXF86AttendantOnXF86AttendantOffXF86AttendantToggleXF86LightsToggleXF86ALSToggleXF86RefreshRateToggleXF86ButtonconfigXF86TaskmanagerXF86JournalXF86ControlPanelXF86AppSelectXF86ScreensaverXF86VoiceCommandXF86AssistantXF86EmojiPickerXF86DictateXF86CameraAccessEnableXF86CameraAccessDisableXF86CameraAccessToggleXF86AccessibilityXF86DoNotDisturbXF86BrightnessMinXF86BrightnessMaxXF86KbdInputAssistPrevXF86KbdInputAssistNextXF86KbdInputAssistPrevgroupXF86KbdInputAssistNextgroupXF86KbdInputAssistAcceptXF86KbdInputAssistCancelXF86RightUpXF86RightDownXF86LeftUpXF86LeftDownXF86RootMenuXF86MediaTopMenuXF86Numeric11XF86Numeric12XF86AudioDescXF863DModeXF86NextFavoriteXF86StopRecordXF86PauseRecordXF86VODXF86UnmuteXF86FastReverseXF86SlowReverseXF86DataXF86OnScreenKeyboardXF86PrivacyScreenToggleXF86SelectiveScreenshotXF86NextElementXF86PreviousElementXF86AutopilotEngageToggleXF86MarkWaypointXF86SosXF86NavChartXF86FishingChartXF86SingleRangeRadarXF86DualRangeRadarXF86RadarOverlayXF86TraditionalSonarXF86ClearvuSonarXF86SidevuSonarXF86NavInfoXF86Macro1XF86Macro2XF86Macro3XF86Macro4XF86Macro5XF86Macro6XF86Macro7XF86Macro8XF86Macro9XF86Macro10XF86Macro11XF86Macro12XF86Macro13XF86Macro14XF86Macro15XF86Macro16XF86Macro17XF86Macro18XF86Macro19XF86Macro20XF86Macro21XF86Macro22XF86Macro23XF86Macro24XF86Macro25XF86Macro26XF86Macro27XF86Macro28XF86Macro29XF86Macro30XF86MacroRecordStartXF86MacroRecordStopXF86MacroPresetCycleXF86MacroPreset1XF86MacroPreset2XF86MacroPreset3XF86KbdLcdMenu1XF86KbdLcdMenu2XF86KbdLcdMenu3XF86KbdLcdMenu4XF86KbdLcdMenu5SunFA_GraveSunFA_CircumSunFA_TildeSunFA_AcuteSunFA_DiaeresisSunFA_CedillaSunF36SunF37SunSys_ReqSunPropsSunFrontSunCopySunOpenSunPasteSunCutSunPowerSwitchSunAudioLowerVolumeSunAudioMuteSunAudioRaiseVolumeSunVideoDegaussSunVideoLowerBrightnessSunVideoRaiseBrightnessSunPowerSwitchShiftDring_accentDcircumflex_accentDcedilla_accentDacute_accentDgrave_accentDtildeDdiaeresisDRemovehpClearLineClearLinehpInsertLineInsertLinehpDeleteLineDeleteLinehpInsertCharInsertCharhpDeleteCharDeleteCharhpBackTabBackTabhpKP_BackTabKP_BackTabhpModelock1hpModelock2hpResetResethpSystemSystemhpUserUserhpmute_acutemute_acutehpmute_gravemute_gravehpmute_asciicircummute_asciicircumhpmute_diaeresismute_diaeresishpmute_asciitildemute_asciitildehpliralirahpguilderguilderhpYdiaeresishpIOIOhplongminuslongminushpblockblockosfCopyosfCutosfPasteosfBackTabosfBackSpaceosfClearosfEscapeosfAddModeosfPrimaryPasteosfQuickPasteosfPageLeftosfPageUposfPageDownosfPageRightosfActivateosfMenuBarosfLeftosfUposfRightosfDownosfEndLineosfBeginLineosfEndDataosfBeginDataosfPrevMenuosfNextMenuosfPrevFieldosfNextFieldosfSelectosfInsertosfUndoosfMenuosfCancelosfHelposfSelectAllosfDeselectAllosfReselectosfExtendosfRestoreosfDeleteExt16bit_LExt16bit_R";

pub(super) static KEYSYM_TO_IDX: IndexMap<u32, u16> = IndexMap {
    key: 12913932095322966823,
    disps: &[(0, 3), (1, 695), (0, 104), (0, 88), (1, 2181), (0, 27), (0, 97), (0, 392), (0, 8), (0, 30), (0, 1087), (0, 287), (0, 2), (0, 2), (0, 9), (0, 0), (0, 50), (0, 37), (0, 72), (0, 4), (0, 418), (0, 563), (0, 398), (0, 1567), (0, 533), (0, 28), (0, 434), (0, 24), (0, 38), (0, 1160), (0, 1), (0, 131), (0, 108), (0, 1), (0, 3), (0, 26), (0, 564), (0, 16), (0, 0), (0, 9), (0, 1), (0, 1), (0, 1204), (0, 0), (0, 4), (0, 1890), (0, 0), (0, 142), (2, 252), (0, 28), (0, 49), (0, 5), (0, 15), (0, 1), (0, 56), (0, 2), (0, 45), (0, 2180), (0, 2), (0, 36), (0, 1), (0, 0), (0, 27), (0, 81), (0, 0), (0, 2), (0, 85), (0, 0), (0, 1442), (0, 118), (0, 2), (0, 14), (0, 0), (0, 0), (0, 17), (0, 0), (0, 0), (0, 2), (0, 0), (0, 2), (0, 1), (0, 0), (0, 0), (0, 2), (0, 1), (0, 0), (0, 58), (0, 0), (0, 0), (0, 24), (0, 1), (0, 0), (0, 62), (0, 0), (0, 0), (0, 3), (0, 11), (0, 40), (0, 0), (0, 60), (0, 0), (0, 20), (0, 3), (0, 495), (0, 39), (0, 13), (0, 4), (0, 1528), (0, 1190), (0, 1441), (0, 218), (0, 147), (0, 1182), (0, 196), (0, 11), (0, 1), (0, 179), (0, 20), (0, 22), (0, 30), (0, 641), (0, 78), (0, 2445), (1, 724), (0, 535), (0, 1236), (0, 65), (0, 837), (0, 50), (0, 65), (0, 216), (0, 69), (0, 401), (0, 7), (0, 3), (0, 151), (0, 454), (0, 5), (0, 27), (0, 45), (0, 0), (0, 16), (0, 4), (0, 6), (0, 294), (0, 20), (0, 108), (0, 494), (0, 2), (0, 435), (0, 346), (0, 23), (0, 1619), (0, 197), (0, 16), (0, 1657), (0, 101), (0, 648), (0, 168), (0, 8), (1, 1065), (0, 5), (0, 15), (0, 45), (0, 2), (0, 45), (0, 2), (0, 49), (0, 3), (0, 24), (0, 22), (0, 11), (0, 306), (0, 22), (1, 429), (0, 52), (0, 19), (0, 1019), (0, 905), (1, 720), (0, 139), (0, 265), (0, 141), (0, 19), (0, 2), (0, 3), (0, 2), (0, 1576), (0, 5), (0, 5), (0, 6), (0, 715), (0, 2138), (0, 22), (1, 86), (0, 176), (0, 16), (0, 55), (0, 862), (4, 2141), (0, 0), (0, 4), (0, 1998), (0, 2), (0, 32), (0, 464), (0, 0), (1, 1051), (0, 190), (0, 2), (0, 1), (0, 21), (0, 17), (0, 95), (0, 10), (0, 86), (0, 113), (0, 53), (0, 113), (0, 91), (0, 178), (0, 121), (0, 45), (0, 18), (0, 17), (0, 15), (0, 141), (0, 0), (0, 1), (0, 41), (0, 1023), (0, 0), (0, 19), (0, 863), (0, 55), (0, 3), (0, 2), (0, 11), (0, 1), (0, 95), (0, 57), (0, 0), (0, 23), (0, 0), (0, 0), (1, 132), (0, 209), (1, 2254), (0, 6), (0, 0), (0, 1), (0, 3), (0, 2), (0, 2), (0, 62), (0, 0), (0, 182), (0, 1), (0, 0), (0, 13), (0, 70), (0, 1), (0, 1493), (0, 1), (0, 5), (0, 79), (0, 17), (0, 750), (0, 813), (0, 208), (0, 41), (0, 9), (0, 1), (0, 1), (0, 57), (0, 11), (2, 2060), (2, 296), (0, 1), (0, 1410), (0, 295), (0, 2363), (0, 505), (0, 612), (0, 1188), (0, 1), (0, 39), (1, 50), (0, 233), (0, 15), (0, 12), (0, 447), (0, 455), (11, 580), (4, 524), (0, 615), (3, 1411), (0, 21), (0, 31), (1, 1693), (0, 294), (0, 1070), (2, 176), (0, 380), (0, 3), (0, 2), (1, 2173), (0, 15), (0, 16), (1, 255), (0, 1), (0, 0), (0, 235), (0, 3), (0, 17), (3, 1527), (0, 418), (0, 0), (0, 256), (0, 20), (0, 587), (0, 72), (1, 388), (1, 348), (0, 13), (0, 1120), (9, 1906), (0, 98), (0, 153), (0, 62), (0, 5), (0, 149), (0, 0), (0, 9), (0, 344), (0, 113), (0, 132), (2, 1609), (0, 44), (0, 39), (0, 350), (0, 456), (21, 841), (0, 744), (0, 28), (0, 5), (8, 1357), (0, 119), (0, 703), (0, 8), (6, 2088), (0, 1365), (0, 4), (0, 267), (0, 233), (0, 9), (0, 0), (0, 4), (0, 5), (0, 0), (0, 70), (9, 911), (0, 740), (0, 467), (3, 1475), (2, 132), (0, 397), (0, 1148), (0, 92), (1, 2362), (0, 341), (0, 0), (0, 97), (4, 1258), (0, 2), (0, 712), (0, 292), (0, 49), (0, 560), (0, 16), (0, 7), (0, 23), (0, 211), (0, 133), (0, 86), (0, 2359), (4, 946), (0, 320), (0, 1311), (0, 1743), (0, 16), (0, 15), (3, 1630), (0, 4), (0, 15), (4, 1692), (0, 643), (0, 28), (0, 500), (0, 12), (0, 66), (0, 121), (0, 3), (0, 35), (0, 3), (0, 0), (0, 433), (0, 130), (0, 0), (0, 90), (0, 1089), (0, 0), (0, 781), (0, 13), (0, 0), (0, 1464), (0, 0), (0, 45), (0, 93), (0, 329), (0, 42), (0, 30), (0, 373), (0, 204), (0, 2303), (0, 1), (0, 1), (0, 7), (0, 0), (0, 0), (0, 18), (0, 379), (0, 0), (2, 1071), (0, 2), (0, 0), (10, 2282), (0, 7), (0, 144), (3, 1665), (0, 17), (0, 0), (0, 2162), (0, 1), (0, 306), (0, 304), (0, 236), (30, 2291), (0, 2280), (0, 10), (3, 927), (0, 131), (1, 358), (0, 13), (0, 1601), (0, 202), (0, 0), (1, 536), (1, 81), (0, 472), (0, 1), (1, 383), (0, 943), (109, 892), (4, 1803), (174, 1956), (4, 1749), (14, 670), (0, 155), (0, 684), (20, 912), (1, 722), (0, 12), (0, 1), (14, 117), (0, 87), (0, 183), (0, 1044), (0, 109), (21, 444), (0, 10), (0, 66), (0, 2), (0, 25), (0, 4), (0, 1520), (0, 7), (11, 471), (0, 33), (0, 33), (2, 1793), (0, 141), (3, 34), (7, 562), (2, 1634), (0, 261), (0, 2302), (5, 2269), (0, 97), (14, 1811), (1, 179), (0, 16), (2, 653), (0, 6), (0, 142), (0, 114), (0, 2), (0, 24), (0, 282), (1, 1739), (3, 649), (0, 133)],
    map: &[1752, 2057, 82, 2164, 1440, 2383, 842, 2170, 536, 1180, 204, 836, 1974, 1226, 1905, 930, 1356, 1712, 2434, 1643, 643, 2111, 174, 2445, 2232, 1763, 741, 754, 696, 2277, 1858, 1607, 745, 2135, 1553, 955, 1221, 1282, 411, 1623, 1099, 1605, 755, 1573, 2400, 2533, 441, 2126, 1350, 2324, 900, 2297, 2204, 9, 753, 378, 210, 1421, 2567, 1309, 2331, 1904, 2384, 1766, 1359, 2043, 1063, 243, 2354, 1498, 2487, 1927, 601, 1928, 1010, 2047, 604, 1860, 2369, 1842, 1000, 1545, 817, 303, 1911, 2253, 1247, 892, 2544, 418, 1405, 771, 1936, 1146, 2355, 2293, 257, 553, 2141, 503, 62, 2061, 2564, 735, 670, 154, 683, 1529, 653, 115, 2537, 2009, 1154, 2279, 1023, 1608, 447, 1748, 363, 2485, 2261, 1260, 1269, 1166, 1081, 924, 1422, 953, 1625, 254, 2390, 2088, 104, 2236, 1910, 485, 806, 1751, 1682, 521, 1332, 543, 635, 297, 1855, 2330, 2153, 1299, 1097, 2026, 603, 954, 1823, 1658, 812, 1812, 1795, 873, 1612, 835, 1934, 1225, 743, 194, 371, 416, 2339, 1818, 2407, 2089, 101, 2470, 1848, 1520, 478, 337, 1620, 1, 2282, 1505, 1138, 285, 2512, 1041, 921, 1349, 699, 406, 558, 344, 878, 2296, 2559, 1544, 1092, 319, 289, 1659, 2015, 2320, 1883, 2353, 2252, 982, 1111, 1193, 2027, 467, 2046, 2374, 1946, 2306, 2413, 1412, 123, 2176, 778, 899, 1710, 1279, 1775, 1621, 1997, 999, 2072, 1582, 2486, 1114, 1381, 891, 345, 1850, 1153, 471, 652, 1208, 1199, 1841, 2016, 1681, 2388, 1460, 1679, 1170, 1171, 2060, 100, 1389, 510, 738, 2116, 152, 606, 1484, 1080, 2319, 529, 377, 1819, 385, 1390, 730, 805, 1507, 569, 1120, 906, 1339, 2484, 1916, 1268, 2165, 1246, 973, 2033, 2401, 2453, 2389, 362, 849, 2087, 103, 484, 2152, 463, 4, 2220, 1254, 971, 541, 561, 1039, 2329, 317, 1611, 1458, 1660, 702, 2258, 2557, 1364, 2307, 2451, 2193, 1093, 1508, 215, 1289, 612, 205, 1933, 8, 1986, 382, 1227, 2423, 969, 2419, 2270, 25, 559, 415, 1774, 2235, 1811, 1394, 1316, 759, 31, 2535, 1137, 1583, 466, 2566, 1452, 841, 1379, 1892, 1734, 332, 1083, 2281, 1758, 118, 822, 2014, 1699, 261, 2352, 1646, 216, 1169, 1494, 2003, 2373, 714, 1863, 2558, 581, 1567, 1756, 122, 1052, 777, 1203, 2078, 432, 1214, 804, 2044, 307, 2168, 2259, 1411, 1337, 1768, 1648, 1594, 453, 1554, 1086, 324, 464, 597, 2031, 1714, 796, 1480, 1152, 14, 2172, 2541, 528, 1962, 2406, 1040, 2422, 2115, 1757, 2498, 2257, 405, 176, 540, 1593, 1082, 1096, 1687, 1906, 1963, 1385, 904, 1338, 2093, 110, 2574, 925, 1496, 611, 1980, 509, 2032, 1315, 1825, 2226, 848, 701, 2175, 1915, 557, 361, 1476, 1192, 881, 976, 2345, 442, 2221, 3, 715, 2576, 1610, 1257, 1014, 1187, 1363, 896, 1781, 1434, 1506, 782, 246, 1882, 2240, 1517, 1893, 269, 1194, 392, 381, 150, 109, 1688, 2457, 580, 30, 988, 247, 1481, 929, 1355, 665, 800, 1945, 1463, 1374, 1985, 1767, 710, 2380, 422, 2548, 374, 1378, 2107, 978, 2051, 2192, 1801, 871, 1110, 1772, 1698, 952, 1027, 1451, 781, 1899, 2002, 1615, 649, 1202, 1005, 2222, 638, 306, 367, 2522, 2494, 130, 776, 656, 2394, 547, 2157, 722, 671, 2440, 1380, 923, 2065, 87, 869, 1425, 568, 1410, 2571, 1479, 1600, 182, 1563, 708, 397, 1085, 164, 747, 2120, 2370, 1799, 2187, 1827, 1277, 1551, 1303, 740, 404, 2496, 810, 1729, 1745, 490, 2411, 2186, 1800, 620, 36, 1779, 1552, 1459, 323, 1626, 2077, 746, 958, 271, 2276, 2359, 201, 590, 2188, 1733, 1432, 1802, 1635, 1581, 1950, 1922, 895, 565, 1939, 1488, 2471, 1123, 2224, 2344, 1160, 870, 2378, 260, 1013, 1433, 2129, 2050, 1670, 1864, 2096, 127, 766, 1399, 1141, 221, 1416, 758, 623, 2280, 70, 788, 1504, 1317, 1046, 349, 2140, 1898, 273, 832, 709, 1705, 2199, 433, 1865, 2358, 470, 421, 1846, 1175, 1992, 534, 686, 1026, 292, 721, 1735, 1417, 514, 2313, 2379, 1828, 989, 1598, 690, 1728, 2180, 1587, 693, 1869, 1870, 2427, 1343, 489, 2472, 2333, 394, 1968, 248, 2520, 277, 2037, 599, 2393, 853, 2156, 1851, 2450, 2439, 1845, 181, 1515, 207, 268, 2128, 987, 2305, 2139, 10, 1302, 2076, 546, 1599, 272, 739, 1561, 1998, 2563, 674, 1004, 1868, 589, 584, 1122, 278, 614, 1674, 2064, 86, 600, 1990, 2410, 1744, 1102, 938, 864, 2106, 128, 1778, 366, 1398, 1815, 2263, 1881, 1384, 2020, 203, 1319, 1019, 2275, 1921, 265, 706, 1518, 220, 1291, 1645, 1634, 440, 446, 1703, 1549, 462, 1938, 1692, 197, 2007, 1159, 2473, 2377, 513, 1788, 1276, 993, 1213, 2542, 192, 1949, 2338, 274, 2146, 791, 360, 1140, 2066, 632, 2416, 93, 1649, 1967, 291, 2300, 1685, 765, 1220, 454, 387, 1369, 1829, 2019, 1694, 1654, 2173, 1103, 1090, 2459, 1232, 1798, 1174, 1075, 2465, 532, 234, 1539, 1370, 2506, 658, 2163, 797, 497, 2083, 114, 764, 1068, 1586, 350, 1537, 2289, 1342, 55, 1034, 206, 2325, 2274, 1415, 774, 959, 2036, 188, 1564, 399, 330, 452, 1541, 733, 1764, 2554, 1297, 2466, 410, 705, 2273, 2349, 6, 852, 2123, 626, 1739, 1438, 276, 964, 2562, 2230, 1704, 975, 663, 1817, 336, 1719, 936, 1235, 1283, 2245, 474, 2223, 1989, 1903, 1326, 2347, 1429, 1132, 2262, 913, 1091, 439, 2040, 785, 720, 886, 1669, 299, 1530, 318, 312, 897, 667, 2368, 2299, 726, 862, 1721, 138, 2133, 858, 1617, 1455, 458, 1966, 338, 435, 564, 1576, 457, 2006, 384, 1525, 1663, 1251, 995, 37, 1320, 2417, 1932, 1009, 1198, 2526, 915, 1357, 2381, 2382, 1179, 1206, 792, 991, 92, 535, 2203, 1973, 2110, 172, 2056, 2433, 2323, 1516, 1717, 335, 45, 1604, 1467, 645, 1804, 695, 1857, 476, 2464, 828, 488, 1926, 1281, 2549, 879, 811, 2504, 1979, 2547, 1032, 2399, 1956, 1673, 682, 1887, 1067, 187, 2288, 826, 752, 795, 355, 225, 2095, 126, 1571, 1816, 2082, 1033, 437, 1771, 2196, 1562, 427, 329, 1308, 2145, 1885, 1765, 618, 1943, 2162, 496, 1550, 2493, 576, 241, 2209, 2553, 47, 1874, 2229, 571, 1116, 1651, 2100, 666, 816, 1462, 2462, 1145, 1523, 2174, 867, 2309, 296, 1941, 1902, 1373, 1106, 1457, 1640, 1219, 1639, 1456, 12, 1388, 1875, 1835, 885, 1709, 2455, 1307, 409, 1996, 1230, 1143, 2055, 90, 1224, 1750, 1062, 136, 1528, 2314, 1787, 877, 1691, 426, 724, 1058, 2477, 2458, 1575, 428, 2237, 113, 354, 1325, 1657, 1852, 1720, 831, 1618, 1222, 1952, 1404, 2025, 311, 662, 550, 520, 1859, 1478, 2565, 1972, 840, 2202, 91, 2109, 1331, 1822, 2348, 170, 2432, 725, 1089, 947, 1762, 2250, 238, 2122, 2350, 2295, 1856, 2215, 1503, 284, 637, 608, 2242, 1925, 770, 1475, 1978, 1348, 2398, 494, 2241, 275, 2478, 861, 2119, 1833, 1569, 1240, 7, 1747, 408, 751, 1664, 224, 2444, 629, 251, 1474, 698, 1021, 1444, 1638, 1280, 1873, 588, 920, 1686, 1526, 617, 240, 2045, 83, 226, 2530, 1840, 288, 1672, 1259, 815, 501, 2228, 890, 1427, 1118, 1403, 40, 1001, 1531, 1163, 1472, 992, 2150, 712, 2366, 1653, 2492, 1324, 228, 1603, 1236, 2569, 1538, 944, 316, 46, 1072, 2269, 1969, 2132, 1655, 1793, 1383, 1164, 99, 1668, 2430, 2412, 2387, 2483, 1749, 1708, 1266, 2363, 2197, 483, 2404, 2255, 1223, 1442, 679, 1435, 951, 664, 970, 2421, 475, 1862, 2212, 2249, 2328, 1821, 994, 519, 1095, 2151, 2024, 575, 607, 839, 2086, 102, 2023, 1557, 1197, 1249, 998, 876, 1288, 1942, 295, 2337, 252, 1955, 414, 1810, 1742, 373, 1761, 2008, 2234, 2405, 1244, 1501, 2479, 2431, 1073, 1879, 1055, 1144, 507, 2246, 551, 1805, 282, 38, 981, 857, 178, 1944, 1590, 456, 1838, 1420, 2013, 1439, 1314, 2552, 214, 1572, 1168, 1725, 342, 1891, 713, 1126, 1330, 1755, 2213, 121, 2190, 1628, 593, 1445, 1151, 1652, 1533, 1791, 1020, 1443, 2136, 1580, 943, 359, 1336, 200, 2239, 53, 814, 1839, 302, 889, 654, 983, 1702, 2386, 1056, 845, 1662, 508, 1183, 527, 2138, 390, 1195, 2437, 863, 2114, 148, 1536, 2149, 315, 1592, 1191, 59, 1077, 1060, 2476, 950, 444, 2551, 1991, 2030, 287, 2217, 237, 1353, 1914, 2191, 482, 1983, 610, 834, 2403, 847, 2085, 1961, 98, 1861, 661, 191, 1088, 1995, 2266, 1108, 2268, 968, 386, 984, 625, 1312, 1094, 865, 1490, 1792, 1165, 2, 1078, 1878, 927, 1354, 555, 1037, 1570, 1854, 1931, 1732, 1362, 2198, 380, 1287, 245, 2482, 678, 29, 1984, 578, 2336, 1408, 2081, 2291, 1820, 2371, 2167, 230, 1135, 213, 1054, 888, 2218, 506, 2205, 1006, 259, 2233, 445, 1461, 1245, 17, 903, 1890, 2012, 1061, 430, 2069, 54, 1959, 1644, 1524, 1107, 524, 802, 2059, 1313, 821, 1329, 1212, 120, 2514, 2189, 556, 1150, 1264, 1559, 193, 734, 2491, 997, 1695, 823, 1335, 1113, 358, 644, 1377, 1540, 775, 688, 107, 2029, 1409, 707, 162, 844, 2385, 1609, 301, 827, 596, 525, 2010, 1231, 2021, 42, 2436, 2113, 146, 2572, 1059, 538, 196, 372, 1591, 2092, 108, 403, 757, 1051, 1352, 1511, 1148, 270, 1809, 1256, 368, 1131, 2159, 1982, 605, 198, 2210, 1084, 1880, 1038, 1211, 2137, 539, 680, 1579, 2267, 305, 2343, 212, 974, 2489, 2227, 1101, 233, 1468, 2185, 719, 1361, 641, 375, 413, 621, 926, 1182, 2546, 1513, 1884, 267, 2049, 1913, 2042, 2448, 1897, 619, 348, 1426, 1680, 1930, 1871, 27, 41, 227, 1999, 1407, 1886, 2357, 1731, 2414, 786, 1057, 505, 809, 685, 85, 1025, 756, 449, 1376, 231, 2321, 244, 630, 199, 2539, 2011, 1003, 595, 258, 469, 1909, 379, 1960, 365, 2304, 1597, 689, 1449, 1656, 1079, 2518, 1424, 487, 1185, 1547, 2372, 304, 2155, 2063, 67, 1797, 180, 1753, 1200, 1334, 940, 1255, 1650, 744, 1697, 2322, 1301, 195, 321, 642, 2461, 341, 1713, 843, 1386, 583, 1867, 996, 837, 1272, 1218, 1740, 2341, 402, 2426, 1262, 737, 1492, 391, 1548, 1121, 1743, 2409, 1738, 1777, 1397, 1431, 2284, 1667, 901, 1631, 585, 1043, 563, 2278, 1450, 264, 1622, 1368, 1074, 1661, 1633, 1128, 1542, 1794, 1509, 502, 1937, 1619, 419, 2318, 1296, 393, 1158, 1473, 1826, 2376, 1666, 1228, 2178, 2500, 2094, 125, 1414, 1920, 35, 1641, 1155, 512, 2310, 2545, 2214, 266, 807, 2048, 81, 1248, 2127, 1843, 448, 1387, 1136, 763, 1853, 1064, 2018, 1318, 1814, 780, 2073, 1070, 1012, 1448, 808, 2062, 84, 1965, 736, 2238, 1024, 2104, 156, 684, 468, 2091, 1156, 347, 1596, 2538, 1696, 967, 2575, 1919, 545, 364, 1217, 2142, 2244, 1270, 11, 290, 431, 2516, 1718, 2001, 2425, 1759, 544, 1585, 2415, 2356, 2154, 119, 2391, 179, 160, 866, 2272, 217, 2070, 1546, 486, 202, 2074, 105, 851, 1918, 1896, 1736, 1234, 1535, 911, 1866, 829, 60, 401, 1935, 1796, 613, 2340, 2360, 2392, 2166, 2438, 2408, 1300, 1988, 450, 582, 438, 2075, 106, 1770, 1002, 762, 909, 1341, 2285, 2488, 1139, 1018, 1454, 700, 2298, 1724, 631, 2171, 1630, 1737, 218, 1947, 1367, 651, 1189, 622, 1701, 2283, 1786, 1022, 2540, 2005, 1396, 1065, 2375, 1366, 511, 1011, 1447, 1500, 1413, 2102, 1042, 2144, 1205, 562, 2367, 675, 1157, 2207, 2560, 1031, 2256, 2449, 1382, 346, 779, 793, 1115, 1613, 1558, 1716, 310, 598, 1726, 343, 902, 542, 875, 1172, 961, 1683, 2510, 1690, 530, 1803, 1675, 2424, 2103, 2502, 1109, 80, 2463, 2161, 1948, 956, 1964, 1098, 1624, 186, 646, 2287, 907, 1340, 681, 703, 219, 1560, 1258, 328, 44, 2034, 124, 2194, 1574, 2017, 850, 455, 2243, 2177, 2543, 1295, 1584, 1233, 2271, 1241, 2294, 5, 2570, 1671, 1029, 2260, 22, 1016, 1595, 1365, 2561, 2099, 1502, 1521, 1783, 1614, 232, 986, 1008, 407, 2035, 479, 932, 1987, 369, 1917, 1901, 352, 34, 1954, 884, 1514, 803, 334, 1395, 820, 760, 2361, 1684, 1895, 1894, 731, 1017, 1453, 1627, 1207, 2105, 158, 2108, 134, 2420, 395, 934, 783, 112, 2054, 89, 717, 716, 1773, 957, 425, 2004, 353, 704, 1053, 2475, 33, 1971, 1071, 1512, 798, 333, 1678, 2143, 1824, 2396, 43, 824, 263, 2292, 2169, 309, 1007, 1707, 2201, 2131, 1446, 1647, 2124, 2211, 1556, 2468, 325, 1715, 1776, 1087, 549, 2490, 965, 1030, 1723, 2079, 872, 168, 1924, 1215, 1305, 615, 492, 1977, 1347, 71, 2397, 960, 111, 1125, 1832, 300, 2118, 185, 481, 640, 749, 1908, 1048, 223, 424, 579, 628, 1133, 697, 2160, 1306, 1372, 790, 1637, 250, 2469, 400, 1872, 1700, 732, 420, 882, 2308, 894, 1602, 692, 1117, 587, 2528, 326, 1104, 825, 1391, 616, 2182, 1402, 1204, 2286, 639, 1466, 1471, 0, 1994, 423, 1769, 294, 1464, 1785, 39, 1162, 919, 883, 2000, 1900, 1727, 351, 1173, 1722, 2022, 1323, 1049, 634, 1510, 918, 1177, 650, 1188, 1201, 1028, 2053, 88, 723, 672, 132, 574, 1129, 2248, 2090, 2052, 980, 2346, 1100, 2303, 1784, 1589, 1632, 2195, 794, 2365, 184, 946, 985, 2183, 209, 1184, 2524, 383, 2039, 2080, 2395, 838, 1216, 2251, 1178, 567, 1243, 1953, 2474, 1970, 1519, 1119, 990, 2200, 1190, 2265, 2441, 1601, 691, 166, 531, 2573, 1565, 518, 2158, 1499, 2121, 280, 2446, 830, 573, 1124, 657, 1278, 742, 1976, 1419, 1437, 1746, 2456, 1849, 1847, 1393, 1642, 2117, 1780, 472, 1830, 398, 1923, 2225, 1400, 1047, 322, 602, 566, 789, 768, 249, 1371, 2442, 979, 1636, 1237, 32, 2568, 1274, 50, 465, 1229, 1951, 500, 893, 570, 1907, 1790, 357, 1253, 813, 242, 443, 2429, 2097, 2454, 2181, 286, 624, 1161, 389, 1470, 2068, 480, 183, 293, 673, 1130, 2148, 1616, 1958, 586, 1754, 2317, 1534, 1522, 314, 515, 491, 977, 239, 949, 1322, 2550, 58, 592, 1477, 516, 1242, 2364, 856, 2481, 1196, 499, 1834, 1250, 2428, 1940, 1418, 208, 1629, 1760, 1293, 1588, 694, 1993, 190, 2578, 1344, 2219, 2402, 1568, 2247, 769, 1176, 767, 2038, 1482, 2418, 2084, 854, 1706, 2556, 799, 633, 1209, 647, 451, 412, 327, 2335, 2443, 572, 2264, 1127, 1436, 1286, 2071, 660, 1321, 860, 552, 1346, 1406, 917, 1358, 19, 97, 279, 1134, 461, 1844, 2311, 1689, 1210, 1142, 340, 229, 2362, 1238, 1066, 1186, 1050, 2452, 1889, 855, 1836, 370, 1676, 2327, 2509, 1877, 711, 2302, 627, 429, 2301, 523, 1304, 2101, 142, 1045, 728, 669, 548, 1149, 941, 687, 966, 460, 2206, 1578, 2208, 388, 2184, 1693, 1239, 1813, 1555, 69, 308, 1441, 1975, 1789, 2332, 2028, 1486, 537, 1311, 2067, 1181, 1428, 1807, 2179, 1808, 1112, 1167, 2112, 144, 436, 2316, 2254, 676, 1606, 1532, 1485, 313, 1483, 117, 948, 1665, 677, 2435, 2480, 2467, 1351, 609, 1401, 1261, 236, 659, 2577, 498, 1076, 1543, 554, 1469, 262, 1806, 1730, 2147, 189, 1392, 962, 1981, 473, 1035, 963, 1105, 2326, 1566, 2447, 331, 2290, 1837, 1360, 1677, 417, 95, 591, 2555, 942, 874, 256, 1929, 1284, 211, 2334, 887, 972, 2098, 1782, 2130, 818, 504, 1876, 880, 859, 2231, 787, 784, 56, 772, 64, 1147, 1527, 2342, 2125, 1465, 2041, 116, 26, 1912, 339, 1741, 1888, 1375, 1252, 2460, 298, 898, 1957, 594, 1327, 801, 2058, 833, 2508, 1831, 396, 1430, 522, 577, 727, 668, 1711, 222, 1423, 140, 1328, 773, 819, 2351, 459, 2134, 2312, 65, 655, 1577, 1345, 1036, 1333, 2315, 356, 636, 2216, 1310, 320],
    _phantom: core::marker::PhantomData,
};

pub(super) static NAME_TO_IDX: IndexMap<[u8], u16> = IndexMap {
    key: 12913932095322966823,
    disps: &[(0, 6), (0, 0), (0, 1), (0, 32), (0, 125), (0, 0), (0, 46), (0, 23), (0, 29), (0, 10), (0, 3), (0, 136), (0, 36), (0, 5), (0, 4), (0, 5), (0, 5), (0, 11), (0, 0), (0, 3), (0, 4), (0, 1), (0, 6), (0, 3), (0, 17), (0, 2), (0, 0), (0, 5), (0, 37), (0, 2), (0, 0), (0, 1), (0, 0), (0, 4), (0, 0), (0, 86), (0, 6), (0, 2), (0, 26), (0, 32), (0, 1), (0, 4), (0, 2), (0, 0), (0, 7), (0, 2), (0, 12), (0, 0), (0, 23), (0, 22), (0, 0), (0, 0), (0, 9), (0, 25), (0, 19), (0, 14), (0, 0), (0, 0), (0, 12), (0, 66), (0, 0), (0, 13), (0, 12), (0, 83), (0, 2), (0, 13), (0, 6), (0, 12), (0, 0), (0, 2), (0, 0), (0, 0), (0, 23), (0, 2), (0, 22), (1, 2), (0, 5), (0, 0), (0, 5), (0, 0), (0, 0), (0, 4), (0, 0), (0, 3), (0, 10), (0, 2), (0, 1), (0, 0), (0, 0), (0, 8), (0, 6), (0, 59), (0, 0), (0, 1), (0, 24), (0, 7), (0, 25), (0, 83), (0, 1), (0, 1), (0, 26), (0, 10), (0, 13), (0, 42), (0, 3), (0, 3), (0, 50), (0, 8), (0, 0), (0, 4), (0, 0), (0, 6), (0, 53), (0, 0), (0, 0), (0, 36), (0, 7), (0, 29), (0, 7), (0, 1), (0, 6), (0, 3), (0, 11), (0, 9), (0, 9), (0, 0), (0, 7), (0, 182), (0, 42), (0, 0), (0, 24), (0, 2), (0, 21), (0, 3), (0, 0), (0, 4), (0, 1), (0, 2), (0, 21), (0, 26), (0, 9), (0, 2), (0, 0), (0, 0), (0, 26), (0, 16), (0, 7), (0, 9), (0, 9), (0, 22), (0, 1), (0, 4), (0, 2), (0, 139), (0, 286), (0, 29), (0, 19), (0, 1), (0, 0), (0, 1), (0, 0), (0, 0), (0, 22), (0, 0), (0, 2), (0, 192), (0, 4), (0, 0), (0, 19), (0, 0), (0, 54), (0, 17), (0, 101), (0, 17), (0, 21), (0, 7), (0, 32), (0, 15), (0, 5), (0, 2), (0, 2), (0, 5), (0, 33), (0, 58), (0, 0), (0, 10), (0, 1), (0, 57), (0, 22), (0, 64), (0, 4), (0, 1), (0, 19), (0, 6), (0, 42), (0, 0), (0, 5), (0, 0), (0, 0), (0, 18), (0, 30), (0, 410), (0, 0), (0, 33), (0, 12), (0, 2), (0, 3), (0, 1), (0, 0), (0, 0), (0, 1), (0, 8), (0, 1), (0, 141), (0, 3), (0, 35), (0, 0), (0, 2), (0, 26), (0, 0), (0, 7), (0, 1), (0, 6), (0, 8), (0, 0), (0, 11), (0, 7), (0, 337), (0, 3), (0, 2), (0, 9), (0, 21), (0, 23), (0, 0), (0, 7), (0, 22), (0, 0), (0, 0), (0, 14), (0, 4), (0, 104), (0, 1), (0, 68), (0, 1), (0, 0), (0, 0), (0, 277), (0, 22), (0, 115), (0, 51), (0, 0), (0, 2), (0, 6), (0, 5), (0, 197), (0, 0), (0, 7), (0, 11), (0, 2), (0, 1), (0, 0), (0, 2), (0, 1), (0, 2), (0, 800), (0, 2), (0, 4), (0, 77), (0, 17), (0, 5), (0, 5), (0, 4), (0, 28), (0, 3), (0, 6), (0, 1), (0, 13), (0, 7), (0, 11), (0, 18), (0, 0), (0, 27), (0, 481), (0, 18), (0, 6), (0, 1), (0, 12), (0, 0), (0, 7), (0, 20), (0, 1), (0, 2), (0, 3), (0, 54), (0, 0), (0, 1), (0, 2), (0, 140), (0, 10), (0, 13), (0, 5), (0, 13), (0, 5), (0, 92), (0, 16), (0, 730), (0, 10), (0, 6), (0, 4), (0, 79), (0, 2), (0, 11), (0, 0), (0, 5), (0, 11), (0, 0), (0, 1), (0, 8), (0, 2), (0, 756), (0, 0), (0, 0), (0, 10), (0, 3), (0, 6), (0, 52), (0, 13), (0, 54), (0, 2), (0, 0), (0, 1), (0, 10), (0, 0), (0, 33), (0, 14), (0, 33), (0, 3), (0, 7), (0, 19), (0, 5), (0, 0), (0, 7), (0, 0), (0, 433), (0, 8), (0, 0), (0, 0), (0, 37), (0, 4), (0, 7), (0, 43), (0, 0), (0, 1), (0, 5), (0, 99), (0, 126), (0, 2), (0, 14), (0, 51), (0, 0), (0, 196), (0, 7), (0, 1), (0, 20), (0, 0), (0, 1), (0, 0), (0, 4), (0, 0), (0, 0), (0, 0), (0, 830), (0, 3), (0, 5), (0, 4), (0, 21), (0, 643), (0, 3), (0, 0), (0, 76), (0, 73), (0, 12), (0, 0), (0, 59), (0, 1), (0, 17), (0, 3), (0, 78), (0, 9), (0, 98), (0, 3), (0, 10), (0, 4), (0, 52), (0, 0), (0, 5), (0, 203), (0, 0), (0, 8), (0, 32), (0, 76), (0, 75), (0, 0), (0, 9), (0, 24), (0, 35), (0, 15), (0, 0), (0, 81), (0, 26), (0, 3), (0, 3), (0, 12), (0, 16), (0, 2), (0, 907), (0, 2), (0, 67), (0, 20), (0, 684), (0, 1), (0, 8), (0, 7), (0, 142), (0, 0), (0, 118), (0, 50), (0, 0), (0, 0), (0, 1), (0, 6), (0, 0), (0, 15), (0, 3), (0, 1), (0, 29), (0, 1), (0, 4), (0, 6), (0, 116), (0, 17), (0, 207), (0, 6), (0, 1), (0, 24), (0, 2), (0, 1), (0, 7), (0, 0), (0, 8), (0, 0), (0, 5), (0, 57), (0, 2), (0, 6), (0, 49), (0, 2), (0, 82), (0, 1), (0, 3), (0, 4), (0, 8), (0, 7), (0, 59), (0, 5), (0, 5), (0, 19), (0, 2), (0, 1), (0, 269), (0, 0), (0, 0), (0, 82), (0, 12), (0, 3), (0, 15), (0, 547), (0, 57), (0, 43), (0, 0), (0, 0), (0, 22), (0, 22), (0, 357), (0, 2), (0, 15), (0, 0), (0, 0), (0, 7), (0, 1), (0, 15), (0, 0), (0, 77), (0, 0), (0, 11), (0, 0), (0, 15), (0, 0), (0, 71), (0, 47), (0, 17), (0, 0), (0, 6), (0, 5), (0, 1), (0, 13), (0, 4), (0, 0), (0, 10), (0, 2), (0, 46), (0, 124), (0, 18), (0, 952), (0, 24), (0, 0), (0, 0), (0, 12), (0, 74), (0, 4), (0, 10), (0, 100), (0, 7), (0, 0), (0, 3), (0, 6), (0, 22), (0, 27), (0, 26), (0, 27), (0, 3), (0, 326), (0, 3), (0, 8), (0, 16), (0, 51), (0, 1), (0, 16), (0, 33), (0, 34), (0, 5), (0, 7), (0, 0), (0, 1), (0, 0), (0, 9), (0, 0), (0, 9), (0, 10), (0, 67), (0, 0), (0, 45), (0, 15), (0, 0), (0, 5), (0, 4), (0, 53), (0, 0), (0, 3), (0, 0), (0, 7), (0, 41), (0, 0), (0, 0), (0, 72), (0, 224), (0, 204), (0, 38), (0, 1064), (0, 29), (0, 18), (0, 0), (0, 331), (0, 0), (0, 617), (0, 0), (0, 5), (0, 33), (0, 7), (0, 54), (0, 13), (0, 13), (0, 1), (0, 0), (0, 14), (0, 156), (0, 19), (0, 2), (0, 53), (0, 7), (0, 781), (0, 0), (0, 1), (0, 123), (0, 0), (0, 767), (0, 4), (0, 0), (0, 0), (0, 0), (0, 0), (0, 16), (0, 118), (0, 866), (0, 0), (0, 554), (0, 0), (0, 59), (0, 125), (0, 16), (0, 89), (0, 42), (0, 283), (0, 2), (0, 80), (0, 19), (0, 50), (0, 76), (0, 0), (0, 12), (0, 23), (0, 0), (0, 281), (0, 539), (0, 0), (0, 3), (0, 0), (0, 1), (0, 0), (0, 413), (0, 13), (0, 3), (0, 0), (0, 3), (0, 310), (0, 37), (0, 2), (0, 0), (0, 75), (0, 66), (0, 34), (0, 0), (0, 14), (0, 241), (0, 0), (0, 25), (0, 32), (0, 9), (0, 3), (0, 10), (0, 127), (0, 0), (0, 98), (0, 0), (0, 12), (0, 74), (0, 0), (0, 10), (0, 4), (0, 31), (0, 8), (0, 562), (0, 12), (0, 646), (0, 0), (0, 17), (0, 0), (0, 29), (0, 5), (0, 0), (0, 12), (0, 136), (0, 1110), (0, 3), (0, 124), (0, 543), (0, 45), (0, 0), (0, 0), (0, 31), (0, 1), (0, 5), (0, 3), (0, 168), (0, 15), (0, 11), (0, 29), (0, 9), (0, 6), (0, 10), (0, 50), (0, 3), (0, 1), (0, 0), (0, 28), (0, 8), (0, 0), (0, 2), (0, 145), (0, 0), (0, 13), (0, 9), (0, 0), (0, 66), (0, 52), (0, 3), (0, 11), (0, 16), (0, 132), (0, 0), (0, 67), (0, 0), (0, 23), (0, 24), (0, 0), (0, 19), (0, 6), (0, 20), (0, 0), (0, 14), (0, 16), (0, 98), (0, 6), (0, 7), (0, 50), (0, 9), (0, 0), (0, 7), (0, 10), (0, 15), (0, 586), (0, 305), (0, 33), (0, 12), (0, 3), (0, 10), (0, 21), (0, 5), (0, 250), (0, 0), (0, 25), (0, 55), (0, 5), (0, 32), (0, 43), (0, 47), (0, 7), (0, 44), (0, 13), (0, 0), (0, 11), (0, 641), (0, 19), (0, 59), (0, 360), (0, 1), (0, 3), (0, 19), (0, 4), (0, 226), (0, 824), (0, 9), (0, 86), (0, 1), (0, 6), (0, 0), (0, 321), (0, 4), (0, 1), (0, 6), (0, 99), (0, 13), (0, 95), (0, 38), (0, 492), (0, 1), (0, 0), (0, 10), (0, 8), (0, 800), (0, 238), (0, 0), (0, 77), (0, 17), (0, 34), (0, 78), (0, 91), (0, 343), (0, 1319), (0, 39), (0, 31), (0, 74), (0, 0), (0, 2), (0, 1), (0, 11), (0, 0), (0, 0), (0, 3), (0, 2), (0, 2), (0, 11), (0, 8), (0, 1), (0, 884), (0, 27), (0, 1004), (0, 52), (0, 70), (0, 98), (0, 120), (0, 1956), (0, 0), (0, 9), (0, 184), (0, 35), (0, 3), (0, 41), (0, 0), (0, 36), (0, 11), (0, 10), (0, 0), (0, 1378), (0, 0), (0, 0), (0, 13), (0, 62), (0, 0), (0, 82), (0, 10), (0, 21), (0, 3), (0, 0), (0, 1), (0, 10), (0, 49), (0, 2013), (0, 6), (0, 1971), (0, 37), (0, 3), (0, 360), (0, 27), (0, 51), (0, 66), (0, 1010), (0, 1190), (0, 72), (0, 13), (0, 102), (0, 37), (0, 4), (0, 0), (0, 1272), (0, 2), (0, 25), (0, 8), (0, 4), (0, 334), (0, 6), (0, 6), (0, 4), (0, 45), (0, 100), (0, 0), (0, 0), (0, 16), (0, 28), (0, 9), (0, 34), (0, 1), (0, 12), (0, 4), (0, 0), (0, 0), (0, 3), (0, 64), (0, 1), (0, 91), (0, 523), (0, 14), (0, 25), (0, 0), (0, 111), (0, 22), (0, 19), (0, 218), (0, 2), (0, 10), (0, 0), (0, 300), (0, 1), (0, 408), (0, 57), (0, 9), (0, 438), (0, 102), (0, 572), (0, 55), (0, 0), (0, 19), (0, 1), (0, 105), (0, 411), (0, 0), (0, 1504), (0, 7), (0, 0), (0, 34), (0, 86), (0, 0), (0, 116), (0, 245), (0, 202), (0, 26), (0, 13), (0, 291), (0, 8), (0, 0), (0, 0), (0, 1633), (0, 100), (0, 109), (0, 8), (0, 26), (0, 45), (0, 0), (0, 1669), (0, 0), (0, 1698), (0, 11), (0, 40), (0, 0), (0, 4), (0, 0), (0, 93), (0, 717), (0, 83), (0, 0), (0, 10), (0, 2), (0, 157), (0, 11), (0, 959), (0, 270), (0, 30), (0, 81), (0, 0), (0, 0), (0, 1), (0, 23), (0, 1534), (0, 1), (0, 75), (0, 105), (0, 78), (0, 17), (0, 52), (0, 0), (0, 0), (0, 0), (0, 0), (0, 326), (0, 2003), (0, 11), (0, 1), (0, 2090), (0, 26), (0, 2), (0, 1210), (0, 155), (0, 8), (0, 97), (0, 16), (0, 0), (0, 36), (0, 145), (0, 1), (0, 12), (0, 21), (0, 1895), (0, 26), (0, 311), (0, 10), (0, 2), (0, 1322), (0, 0), (0, 0), (0, 18), (0, 33), (0, 818), (0, 1674), (0, 1546), (0, 1765), (0, 802), (0, 19), (0, 697), (0, 13), (0, 2444), (0, 19), (0, 8), (0, 1), (0, 2113), (0, 348), (0, 47), (0, 22), (0, 28), (0, 2412), (0, 0), (0, 842), (0, 10), (0, 21), (0, 244), (0, 4), (0, 0), (0, 4), (0, 199), (0, 1), (0, 49), (0, 897), (0, 1), (0, 52), (0, 2470)],
    map: &[185, 1156, 1606, 2554, 484, 2512, 713, 122, 667, 1460, 1725, 2402, 991, 1400, 1709, 1753, 161, 1345, 482, 1637, 1987, 193, 816, 1409, 1176, 1329, 2055, 83, 1964, 2522, 1008, 590, 1592, 2564, 2536, 2074, 2444, 871, 721, 256, 1997, 1788, 938, 1435, 1022, 2159, 1666, 2513, 843, 1971, 1237, 1234, 2143, 1291, 383, 100, 2266, 105, 518, 1930, 1226, 2062, 359, 1252, 1817, 1274, 1120, 1547, 1525, 1002, 2053, 681, 2345, 2415, 199, 860, 2557, 2571, 1884, 1303, 1708, 1426, 2306, 2386, 1617, 2313, 1625, 1260, 2094, 1768, 971, 2458, 2531, 1982, 1766, 757, 238, 950, 1879, 307, 1883, 961, 859, 1697, 2281, 1653, 1395, 1593, 1669, 1888, 680, 2187, 603, 1146, 642, 1, 165, 2443, 481, 525, 1796, 2370, 241, 417, 925, 1401, 1554, 195, 56, 247, 1147, 655, 2563, 2212, 360, 557, 440, 798, 2279, 2001, 1791, 2121, 652, 988, 1225, 745, 1809, 1747, 457, 1454, 2098, 45, 355, 530, 1534, 1124, 184, 458, 701, 92, 1216, 1053, 1011, 1258, 1312, 956, 815, 1415, 2137, 804, 2518, 1102, 2577, 2056, 609, 205, 1728, 1061, 1513, 196, 836, 282, 400, 1201, 1162, 1033, 1614, 693, 848, 786, 1718, 211, 1326, 1658, 1847, 1172, 1783, 717, 1951, 2100, 1774, 2049, 2562, 163, 1919, 1320, 1036, 770, 116, 2293, 446, 2319, 568, 1555, 880, 479, 1852, 310, 2128, 892, 1993, 478, 2495, 650, 792, 2157, 2058, 2032, 1284, 1272, 2430, 1361, 897, 2542, 42, 940, 1070, 2052, 1380, 1073, 785, 1763, 1464, 1901, 1389, 1623, 2481, 1676, 12, 1642, 277, 2468, 1399, 1218, 2165, 1264, 142, 39, 736, 805, 123, 1889, 910, 2467, 2046, 1085, 1797, 1793, 1052, 270, 349, 1690, 1473, 445, 849, 2543, 2117, 703, 2547, 1671, 1139, 1907, 437, 900, 415, 414, 1004, 1541, 1752, 1414, 1417, 1484, 1836, 1810, 2035, 1664, 793, 962, 570, 251, 688, 2238, 862, 835, 327, 2028, 1698, 1652, 1268, 2560, 2277, 1082, 216, 2144, 1931, 2382, 101, 1136, 1452, 1640, 1410, 839, 2429, 1576, 810, 1092, 1205, 1536, 1895, 2461, 1970, 895, 2363, 2205, 2118, 1222, 2077, 1769, 946, 2109, 1914, 1722, 467, 1685, 1824, 893, 335, 94, 1180, 2453, 1977, 118, 249, 933, 2183, 521, 690, 928, 2322, 2119, 941, 1055, 144, 580, 1319, 1963, 429, 1862, 1119, 1204, 1227, 2368, 1368, 303, 374, 2290, 475, 1301, 72, 855, 17, 1094, 1574, 1250, 233, 656, 1019, 1618, 2357, 326, 2448, 2552, 1953, 113, 1114, 2305, 780, 1242, 1302, 1693, 1539, 2527, 1916, 2332, 1192, 1007, 2379, 1805, 1550, 561, 878, 596, 2263, 523, 2135, 1173, 2107, 2259, 2356, 1923, 2132, 2462, 978, 397, 1035, 662, 2405, 856, 1908, 1042, 1096, 2254, 2505, 1771, 883, 354, 1433, 1476, 460, 1200, 617, 899, 365, 2262, 1168, 2181, 1730, 250, 248, 329, 1751, 1407, 637, 2019, 842, 1832, 261, 1051, 1024, 2457, 979, 372, 346, 1680, 845, 2106, 724, 1518, 1584, 2447, 2501, 2289, 2381, 583, 575, 112, 1679, 612, 1149, 1892, 1445, 390, 162, 13, 213, 558, 546, 854, 2232, 68, 1834, 790, 1758, 830, 2033, 616, 1501, 2469, 358, 1645, 2297, 2026, 2089, 167, 1759, 1029, 1869, 1436, 800, 501, 1027, 515, 1221, 1357, 2437, 1896, 1736, 1880, 2164, 2247, 2010, 646, 442, 818, 1228, 679, 1831, 134, 2421, 74, 2017, 1524, 2488, 1929, 728, 1922, 411, 1438, 923, 2372, 2511, 2393, 1980, 841, 1721, 1840, 2309, 1271, 1582, 469, 2407, 489, 649, 37, 2155, 1208, 1902, 1229, 1595, 718, 86, 1913, 2361, 1261, 2336, 1428, 1504, 198, 2350, 1349, 2075, 81, 2311, 943, 1966, 1451, 972, 826, 215, 208, 389, 789, 795, 2507, 1741, 1724, 753, 2145, 1854, 556, 1280, 1740, 2497, 2426, 1448, 1531, 410, 1799, 2168, 2337, 898, 1077, 739, 1917, 2261, 63, 145, 1122, 1265, 57, 1812, 1209, 119, 1899, 1863, 1846, 1844, 2438, 2242, 2514, 1627, 1808, 1897, 640, 901, 1324, 1144, 819, 2041, 1278, 534, 224, 1018, 686, 2565, 434, 207, 866, 272, 847, 1269, 1905, 2464, 735, 430, 1598, 64, 1005, 768, 1001, 8, 801, 2545, 1240, 48, 1223, 1160, 93, 2496, 2271, 2093, 2317, 1511, 85, 1354, 1494, 1194, 1594, 1622, 40, 2315, 2253, 1925, 691, 25, 831, 426, 1986, 1696, 1232, 1369, 542, 2425, 672, 2071, 319, 1712, 1439, 2069, 540, 1347, 2138, 1998, 613, 1841, 1465, 408, 143, 1878, 1674, 1794, 1466, 2226, 1689, 431, 2413, 1710, 1482, 731, 2530, 934, 1485, 492, 1049, 2111, 439, 26, 850, 1673, 1760, 2286, 2537, 362, 989, 1684, 194, 536, 98, 2307, 503, 1767, 109, 1373, 2136, 1338, 1386, 1773, 1198, 4, 922, 1811, 577, 2005, 2576, 864, 1259, 2516, 628, 2540, 912, 1703, 1057, 2126, 2206, 1542, 1661, 1026, 1402, 1385, 1479, 2535, 2148, 2558, 2556, 217, 425, 1603, 295, 173, 1915, 1965, 188, 1873, 1202, 565, 2285, 407, 711, 2478, 140, 2177, 308, 318, 2246, 949, 2129, 498, 1058, 139, 2011, 915, 149, 2006, 1013, 2369, 873, 1043, 1894, 1145, 1630, 438, 2023, 477, 914, 2045, 1128, 2387, 759, 1254, 1365, 630, 2222, 1211, 386, 384, 2265, 1151, 627, 1081, 765, 2355, 2450, 2329, 1031, 2328, 2394, 677, 1127, 2228, 825, 930, 1546, 108, 2463, 2486, 1288, 1798, 1185, 755, 1499, 531, 2574, 279, 1238, 1334, 619, 1282, 1566, 702, 2433, 1177, 1422, 1683, 2519, 2340, 1827, 1996, 474, 363, 1528, 1243, 2201, 1489, 1110, 1639, 2070, 1044, 514, 259, 1967, 316, 2158, 2528, 2474, 868, 1480, 1688, 1115, 2030, 2042, 1720, 867, 2471, 999, 106, 1558, 297, 919, 1828, 2517, 263, 1244, 1497, 2509, 1850, 423, 1974, 1459, 1782, 660, 2080, 2084, 1442, 1396, 1165, 218, 2061, 1612, 1900, 674, 269, 2503, 2270, 699, 1616, 1348, 1675, 1803, 2323, 385, 473, 1973, 752, 1197, 1134, 1958, 2324, 2449, 1527, 127, 709, 1738, 743, 657, 1100, 1376, 376, 1891, 975, 572, 2087, 1681, 797, 2409, 737, 1549, 2375, 2360, 107, 52, 1629, 283, 2341, 519, 436, 1926, 1178, 1920, 1050, 997, 2215, 1313, 1650, 47, 846, 1084, 2548, 2116, 560, 422, 66, 1155, 2000, 828, 2240, 378, 1336, 2190, 2097, 1597, 1604, 432, 51, 1169, 1983, 356, 1932, 1962, 332, 2399, 1332, 710, 2508, 592, 1286, 1366, 2051, 23, 2110, 2320, 1615, 666, 2316, 1700, 1066, 379, 1350, 1441, 1025, 265, 2170, 2065, 624, 309, 1487, 853, 2434, 982, 992, 352, 275, 2300, 783, 698, 2373, 2066, 1754, 1911, 404, 35, 1719, 834, 727, 32, 2140, 99, 2384, 178, 133, 653, 986, 1571, 239, 345, 172, 2524, 435, 1655, 1421, 2038, 676, 2347, 636, 687, 202, 1475, 694, 1981, 50, 222, 704, 200, 464, 1648, 1101, 19, 1599, 620, 493, 607, 2194, 773, 762, 151, 906, 1732, 336, 2494, 1290, 1838, 1898, 2007, 1562, 746, 15, 1449, 2099, 1735, 2146, 328, 1009, 264, 887, 697, 779, 936, 1755, 2243, 146, 1955, 2358, 1687, 364, 1028, 2213, 766, 1432, 1048, 209, 97, 1750, 929, 500, 339, 714, 38, 733, 104, 665, 1632, 771, 2500, 9, 1075, 1307, 555, 2385, 730, 132, 131, 2400, 876, 1984, 2256, 157, 1991, 1253, 2147, 451, 1111, 2422, 916, 1074, 1412, 891, 2229, 2280, 61, 148, 2544, 1356, 767, 821, 550, 1113, 1656, 1390, 582, 2348, 1083, 1344, 1975, 1737, 716, 324, 598, 331, 1364, 377, 1220, 1581, 2092, 186, 1800, 2423, 966, 242, 817, 1871, 1861, 980, 2490, 2167, 2090, 18, 2248, 1219, 1215, 2161, 1918, 419, 2013, 1537, 754, 281, 2162, 573, 2272, 2460, 2101, 1790, 1686, 1153, 2397, 2114, 1939, 1502, 1126, 1419, 344, 2521, 614, 584, 454, 955, 2250, 24, 749, 1886, 532, 2178, 380, 1493, 375, 894, 450, 1182, 777, 2189, 1694, 46, 337, 1137, 993, 2151, 2029, 313, 175, 1174, 1112, 538, 1784, 1717, 559, 2408, 1672, 2054, 1387, 2480, 1992, 2231, 2489, 2264, 1328, 502, 60, 1453, 554, 2273, 323, 1135, 334, 566, 1857, 998, 1936, 1069, 2268, 1248, 1224, 623, 20, 2219, 2492, 829, 788, 312, 69, 59, 954, 1034, 622, 879, 1327, 1214, 2182, 418, 2354, 1626, 1589, 1533, 1263, 2048, 808, 678, 1020, 2439, 806, 1413, 1446, 587, 2569, 654, 301, 1605, 448, 2362, 1486, 696, 1360, 2330, 529, 2366, 994, 296, 1175, 552, 1780, 1457, 1012, 1474, 1078, 353, 286, 882, 1496, 663, 1251, 382, 2302, 1191, 2538, 1607, 28, 2310, 517, 342, 1408, 2410, 1578, 1804, 201, 2124, 82, 1624, 88, 2466, 1657, 2485, 2199, 750, 2529, 156, 1647, 1179, 1189, 787, 190, 1294, 2498, 486, 2491, 1825, 403, 2003, 719, 516, 1610, 1411, 2440, 416, 553, 191, 2526, 1193, 977, 399, 2570, 1941, 1186, 807, 1706, 2079, 1557, 1379, 276, 2186, 2546, 2388, 1481, 159, 2414, 1040, 1960, 243, 732, 861, 117, 252, 1950, 1946, 121, 599, 931, 1010, 2153, 2572, 747, 447, 1249, 2283, 341, 2208, 1099, 1181, 234, 2396, 1585, 1039, 1551, 2473, 77, 212, 2139, 595, 2294, 1245, 1742, 2367, 1298, 1842, 1801, 2333, 2278, 648, 2091, 1500, 87, 812, 633, 1772, 236, 2255, 103, 2221, 154, 508, 1559, 2103, 373, 1206, 1734, 340, 1940, 948, 549, 2152, 1342, 1512, 953, 995, 1434, 896, 579, 726, 1437, 1887, 497, 124, 1835, 1068, 29, 11, 330, 2034, 1660, 230, 1425, 889, 1279, 210, 2568, 1378, 585, 920, 2236, 192, 1295, 1293, 232, 606, 2371, 764, 401, 2081, 2251, 2060, 1095, 1867, 394, 621, 1404, 1116, 1375, 2076, 1515, 945, 576, 2420, 951, 970, 1556, 463, 1526, 1881, 2292, 811, 2014, 357, 751, 673, 968, 1938, 1450, 756, 2169, 115, 1079, 1978, 2442, 1322, 2113, 791, 758, 905, 601, 203, 1429, 2392, 2504, 268, 581, 2390, 2331, 49, 794, 1678, 351, 796, 2195, 2171, 1785, 1552, 908, 983, 371, 2476, 506, 1470, 1957, 1087, 1483, 2, 952, 904, 187, 1123, 927, 2411, 520, 253, 1505, 1121, 799, 2185, 1532, 44, 1726, 1868, 75, 2441, 1820, 1928, 1406, 2455, 240, 615, 2404, 2291, 302, 393, 1588, 1590, 392, 984, 2002, 1491, 1297, 963, 1323, 1000, 1167, 2150, 453, 2383, 305, 1654, 668, 67, 2218, 1792, 1296, 2352, 2376, 114, 2343, 1520, 2435, 1157, 2156, 1416, 2022, 2359, 1239, 1631, 413, 1985, 644, 823, 348, 1427, 1257, 2334, 2351, 670, 604, 1230, 802, 563, 1601, 2112, 1154, 14, 870, 505, 541, 1667, 2454, 444, 2073, 70, 1954, 742, 513, 96, 2059, 1183, 1062, 5, 1564, 2406, 692, 1021, 921, 1823, 2412, 1776, 2207, 1619, 1864, 2275, 1108, 1067, 2192, 1636, 543, 1275, 1190, 562, 294, 292, 2174, 367, 1393, 2510, 2456, 610, 1677, 2068, 298, 1945, 1138, 1346, 164, 832, 748, 1770, 2502, 1866, 433, 1118, 578, 884, 1702, 1418, 1826, 2039, 2209, 271, 1041, 2123, 1339, 1641, 2082, 347, 1129, 2335, 1553, 512, 1856, 1821, 2416, 2436, 1148, 1130, 1714, 597, 2134, 1535, 547, 1995, 803, 2403, 1638, 537, 645, 509, 2398, 1830, 246, 443, 1731, 1575, 1872, 2031, 2184, 366, 480, 1292, 2267, 2211, 1580, 1775, 1164, 1707, 1644, 1855, 772, 412, 1979, 2276, 522, 1663, 671, 1858, 1472, 594, 2506, 27, 1423, 227, 822, 533, 2204, 1072, 1314, 7, 1132, 608, 2015, 2561, 1522, 1159, 2364, 1093, 1514, 1633, 244, 2050, 643, 837, 31, 1989, 1937, 461, 170, 675, 2088, 2258, 1309, 2377, 885, 1213, 774, 2210, 1276, 2244, 235, 738, 2036, 1161, 120, 2193, 917, 321, 626, 206, 2009, 591, 2483, 1462, 441, 1430, 935, 1545, 1507, 1853, 869, 1382, 1874, 2573, 1814, 574, 391, 1942, 84, 1305, 2288, 2044, 287, 705, 1337, 1490, 471, 763, 2555, 370, 776, 720, 395, 2446, 1089, 2470, 1199, 2142, 1921, 459, 2020, 1086, 2217, 2172, 1374, 169, 1047, 219, 1529, 2321, 73, 2078, 1540, 1384, 1956, 1563, 1568, 1060, 1837, 427, 1388, 1231, 2131, 2551, 1353, 1030, 2083, 632, 1944, 483, 1948, 658, 1340, 1659, 1543, 2326, 1745, 1424, 544, 220, 981, 2027, 1517, 2179, 320, 1890, 136, 1569, 1560, 2086, 280, 625, 1247, 593, 2021, 491, 58, 2378, 350, 629, 1969, 494, 706, 1807, 1961, 2063, 41, 1749, 947, 33, 1600, 2287, 496, 734, 2445, 2298, 1691, 1150, 888, 110, 388, 1152, 1910, 2239, 2318, 1203, 2472, 398, 171, 2534, 78, 315, 2200, 16, 526, 338, 1729, 741, 225, 909, 1577, 2233, 2295, 2431, 844, 2578, 1133, 409, 569, 2566, 2338, 1443, 820, 1933, 715, 669, 926, 1802, 1495, 1538, 1876, 964, 524, 1833, 490, 1649, 1235, 1289, 778, 1723, 2575, 2269, 1105, 2095, 708, 647, 1943, 641, 456, 141, 138, 2180, 535, 278, 53, 1266, 1090, 1561, 2487, 1822, 289, 266, 996, 2515, 1370, 875, 197, 228, 874, 90, 189, 368, 1315, 1586, 1196, 1976, 932, 387, 182, 1596, 1602, 1972, 255, 2417, 2380, 1163, 1088, 1521, 1236, 1744, 1131, 1643, 2260, 293, 2105, 1952, 1359, 2482, 2115, 1635, 1999, 314, 911, 1510, 2173, 2432, 1904, 2559, 965, 1806, 586, 2479, 424, 683, 1158, 661, 2451, 1565, 2339, 2196, 1281, 1579, 1859, 1620, 183, 1818, 2043, 1865, 455, 258, 1262, 2245, 589, 539, 840, 1006, 2520, 2227, 273, 6, 2418, 2057, 174, 1331, 1171, 285, 1316, 111, 1054, 1757, 62, 2374, 2428, 1912, 700, 1544, 2130, 1711, 1273, 1107, 2252, 1355, 877, 1071, 813, 472, 1308, 1233, 1609, 1056, 1241, 2133, 1949, 1065, 1756, 659, 102, 152, 2312, 1306, 1270, 1611, 2427, 1367, 1351, 504, 260, 2365, 1016, 1125, 2025, 588, 2024, 1761, 528, 2072, 1613, 2004, 2085, 317, 54, 551, 1391, 863, 602, 2325, 36, 1333, 1458, 2541, 1246, 291, 176, 1076, 71, 1405, 2008, 2067, 488, 406, 137, 274, 985, 1994, 2499, 290, 1990, 89, 1699, 1665, 1362, 2484, 1447, 1764, 1352, 1064, 1109, 2477, 2223, 1046, 1015, 1704, 1762, 723, 631, 838, 1097, 1572, 1934, 2550, 2149, 1003, 2047, 639, 2203, 468, 1287, 2282, 1716, 973, 1330, 907, 2241, 2235, 1383, 957, 1646, 510, 2016, 969, 858, 1816, 942, 2037, 2301, 150, 2533, 664, 2225, 1032, 1014, 2216, 179, 396, 1335, 2539, 1440, 548, 902, 80, 1909, 2344, 1587, 937, 128, 2175, 1255, 91, 784, 2214, 1217, 300, 1843, 499, 2342, 1285, 722, 1394, 476, 1781, 304, 886, 1463, 153, 1748, 254, 2120, 1141, 564, 1573, 2202, 1210, 1377, 465, 2125, 1343, 1166, 1267, 2346, 695, 2230, 2525, 369, 1117, 1848, 1509, 361, 1506, 267, 3, 1381, 1106, 960, 1143, 744, 1456, 761, 257, 22, 462, 1467, 2127, 43, 1795, 1968, 567, 857, 1787, 284, 0, 177, 223, 944, 2299, 1813, 1692, 1988, 126, 1469, 967, 1695, 1739, 974, 1789, 2465, 1927, 851, 311, 1184, 421, 1188, 76, 452, 30, 1277, 130, 1519, 204, 180, 181, 1420, 1431, 1924, 1371, 2419, 1492, 2304, 1187, 2176, 1885, 635, 2166, 2401, 1321, 2475, 1860, 470, 729, 1882, 611, 381, 1746, 1516, 1398, 2257, 1468, 760, 1829, 299, 306, 924, 2391, 214, 987, 147, 2191, 405, 325, 2424, 322, 618, 1713, 1959, 852, 1743, 1498, 1906, 2040, 1471, 21, 1023, 634, 2104, 1530, 1851, 865, 2064, 1104, 1682, 2249, 2154, 333, 1621, 420, 775, 958, 231, 1091, 1063, 2459, 1372, 782, 600, 2296, 507, 166, 1715, 1662, 740, 1392, 1608, 2012, 1779, 125, 2108, 2389, 1567, 990, 1583, 1140, 229, 2284, 2122, 1503, 55, 343, 1341, 689, 2197, 1819, 1875, 226, 712, 2160, 1310, 1508, 1634, 2220, 2567, 428, 1397, 571, 682, 262, 2188, 707, 1591, 1628, 1651, 487, 1870, 2549, 1523, 1935, 1701, 2096, 1142, 903, 135, 1317, 160, 1170, 939, 769, 1059, 2141, 976, 781, 1304, 2018, 1478, 959, 2523, 1705, 1325, 2102, 1477, 1877, 95, 1778, 1455, 1103, 1358, 918, 402, 2493, 2308, 1098, 245, 466, 1893, 725, 10, 129, 65, 221, 168, 1670, 155, 833, 1903, 1815, 1299, 605, 2349, 1765, 1668, 1311, 1488, 1038, 449, 814, 1195, 2303, 881, 79, 288, 495, 684, 1727, 872, 545, 1786, 824, 2532, 1777, 1256, 1207, 2452, 1548, 638, 1461, 2314, 2353, 1839, 34, 2395, 1037, 527, 1045, 1849, 1283, 1017, 827, 1363, 2198, 1403, 685, 1300, 2163, 1733, 1947, 158, 1570, 913, 237, 511, 1212, 1845, 1080, 1318, 809, 2553, 2237, 651, 485, 2234, 890, 2274, 1444, 2224, 2327],
    _phantom: core::marker::PhantomData,
};

pub(super) static LOWER_NAME_TO_IDX: IndexMap<[u8], u16> = IndexMap {
    key: 12913932095322966823,
    disps: &[(0, 8), (0, 92), (0, 18), (0, 78), (0, 471), (0, 14), (0, 26), (0, 16), (0, 25), (0, 45), (0, 1), (0, 60), (0, 15), (0, 195), (0, 904), (0, 301), (0, 216), (0, 9), (0, 34), (0, 35), (0, 87), (0, 1), (0, 13), (0, 15), (0, 12), (0, 393), (0, 229), (0, 1), (0, 334), (0, 4), (0, 0), (0, 79), (0, 0), (0, 1), (0, 247), (0, 1), (0, 2), (0, 106), (0, 22), (0, 156), (0, 1), (0, 4), (0, 2), (0, 19), (0, 4), (0, 9), (0, 21), (0, 1), (0, 173), (0, 127), (0, 1), (0, 26), (0, 60), (0, 17), (0, 152), (0, 127), (0, 580), (0, 11), (0, 279), (0, 0), (0, 619), (0, 175), (0, 0), (0, 58), (0, 257), (0, 1), (0, 35), (0, 249), (0, 17), (0, 188), (0, 473), (0, 458), (0, 111), (0, 2), (0, 108), (0, 13), (0, 2), (0, 0), (0, 228), (0, 183), (0, 17), (0, 180), (0, 85), (0, 83), (0, 25), (0, 5), (0, 810), (0, 0), (0, 29), (0, 190), (0, 11), (0, 9), (0, 23), (0, 32), (0, 111), (0, 55), (0, 59), (0, 179), (0, 51), (0, 554), (0, 100), (0, 28), (0, 14), (0, 2), (0, 2), (0, 11), (0, 258), (0, 163), (0, 74), (0, 18), (0, 0), (0, 42), (0, 35), (0, 1), (0, 6), (0, 82), (0, 430), (0, 0), (0, 514), (0, 0), (0, 70), (0, 934), (0, 274), (0, 29), (0, 1), (0, 2), (0, 141), (0, 160), (0, 0), (0, 41), (0, 63), (0, 105), (0, 247), (0, 4), (0, 132), (0, 0), (0, 46), (0, 67), (0, 86), (0, 243), (0, 32), (0, 12), (0, 51), (0, 21), (0, 0), (0, 18), (0, 34), (0, 0), (0, 163), (0, 18), (0, 21), (0, 18), (0, 14), (0, 187), (0, 114), (0, 46), (0, 5), (0, 3), (0, 233), (0, 75), (0, 10), (0, 0), (0, 85), (0, 0), (0, 0), (0, 1), (0, 31), (0, 3), (0, 20), (0, 6), (0, 167), (0, 422), (0, 34), (0, 53), (0, 3), (0, 1214), (0, 34), (0, 5), (0, 1374), (0, 4), (0, 22), (0, 212), (0, 56), (0, 40), (0, 634), (0, 28), (0, 0), (0, 58), (0, 1), (0, 0), (0, 17), (0, 899), (0, 28), (0, 29), (0, 186), (0, 12), (0, 1), (0, 112), (0, 12), (0, 2), (0, 15), (0, 0), (0, 0), (0, 221), (0, 23), (0, 20), (0, 1), (0, 0), (0, 243), (0, 24), (0, 4), (0, 581), (0, 192), (0, 122), (0, 83), (0, 50), (0, 549), (0, 2), (0, 56), (0, 448), (0, 6), (0, 24), (0, 8), (0, 3), (0, 15), (0, 5), (0, 564), (0, 266), (0, 10), (0, 320), (0, 210), (0, 0), (0, 168), (0, 7), (0, 23), (0, 5), (0, 0), (0, 115), (0, 87), (0, 159), (0, 20), (0, 26), (0, 160), (0, 137), (0, 77), (0, 202), (0, 0), (0, 2128), (0, 229), (0, 1258), (0, 1), (0, 7), (0, 307), (0, 1), (0, 377), (0, 13), (0, 413), (0, 0), (0, 1386), (0, 0), (0, 4), (0, 1), (0, 4), (0, 2), (0, 147), (0, 61), (0, 1339), (0, 38), (0, 13), (0, 113), (0, 0), (0, 10), (0, 28), (0, 143), (0, 664), (0, 80), (0, 809), (0, 521), (0, 9), (0, 29), (0, 8), (0, 19), (0, 25), (0, 324), (0, 1179), (0, 225), (0, 13), (0, 3), (0, 18), (0, 41), (0, 0), (0, 350), (0, 39), (0, 93), (0, 2025), (0, 728), (0, 6), (0, 55), (0, 229), (0, 846), (0, 1090), (0, 765), (0, 26), (0, 488), (0, 399), (0, 1355), (0, 1346), (0, 974), (0, 12), (0, 81), (0, 37), (0, 1081), (0, 0), (0, 2), (0, 98), (0, 1), (0, 105), (0, 11), (0, 1), (0, 448), (0, 37), (0, 26), (1, 324), (0, 240), (0, 1322), (0, 0), (0, 58), (0, 37), (0, 140), (0, 25), (0, 113), (0, 1092), (0, 1285), (0, 190), (0, 5), (0, 333), (0, 2), (0, 870), (0, 490), (0, 0), (0, 0), (0, 181), (0, 441), (0, 946), (0, 58), (0, 15), (0, 1811), (0, 6), (0, 110), (0, 1004), (0, 42), (0, 113), (0, 95), (0, 43), (0, 466), (0, 1), (0, 11), (0, 2), (0, 173), (0, 1), (0, 847), (0, 144), (0, 108), (0, 0), (0, 61), (0, 952), (0, 1470), (0, 569), (0, 1581), (0, 1), (0, 20), (0, 0), (0, 9), (0, 85), (0, 4), (0, 10), (0, 0), (0, 123), (0, 77), (0, 0), (0, 3), (0, 276), (0, 113), (0, 1), (0, 1150), (0, 670), (0, 481), (2, 562), (0, 234), (0, 78), (0, 329), (0, 79), (0, 32), (0, 2), (0, 11), (0, 0), (0, 86), (0, 150), (0, 469), (0, 423), (0, 20), (0, 1490), (0, 1401), (0, 276), (3, 271), (0, 69), (0, 1277), (0, 1515), (0, 649), (0, 125), (1, 648), (1, 2094), (1, 449), (0, 1526), (0, 8), (0, 1659), (0, 317), (0, 811), (0, 1816), (0, 58), (0, 32), (0, 68), (0, 504), (0, 293), (0, 220), (0, 7), (0, 58), (0, 92), (0, 0), (0, 1031), (0, 47), (0, 0), (1, 82), (0, 176), (0, 0), (0, 39), (0, 902), (0, 35), (0, 292), (1, 46), (0, 56), (0, 179), (0, 118), (0, 93), (0, 13), (0, 673), (0, 3), (0, 2), (0, 706), (0, 2207), (0, 1), (0, 22), (1, 840), (0, 14), (0, 1571), (1, 491), (0, 476), (0, 3), (0, 1001), (0, 514), (0, 143), (0, 5), (0, 72), (0, 776), (0, 0), (0, 230), (0, 117), (0, 14), (0, 4), (0, 223), (0, 110), (0, 124), (0, 35), (0, 827), (1, 365), (1, 1398), (0, 4), (0, 1350), (0, 1392), (1, 1358), (1, 819), (4, 158), (0, 1026), (0, 2), (0, 43), (0, 111), (0, 92), (0, 118), (0, 107), (0, 960), (0, 63), (0, 0), (1, 1634), (0, 20), (0, 2), (0, 12), (0, 237), (0, 1086), (0, 154), (0, 47), (0, 43), (0, 27), (0, 13), (0, 41), (0, 54), (1, 899), (0, 2), (0, 15), (0, 2), (0, 17), (0, 2), (0, 16)],
    map: &[459, 1800, 107, 1275, 249, 1427, 1827, 811, 1252, 1349, 2163, 1861, 2470, 872, 1853, 2218, 2545, 245, 2124, 2166, 1470, 381, 1301, 737, 1306, 310, 2319, 185, 1918, 61, 88, 65, 1465, 73, 219, 1364, 827, 560, 1789, 1959, 1129, 2562, 858, 1474, 280, 1211, 864, 859, 904, 1106, 1412, 2554, 781, 130, 954, 2542, 2322, 1493, 95, 1559, 1551, 1936, 437, 295, 2396, 445, 281, 1629, 335, 1593, 823, 1951, 83, 824, 780, 1281, 2313, 1141, 11, 1627, 1151, 98, 1574, 223, 1280, 299, 2355, 1060, 2366, 2114, 1173, 1701, 1155, 1529, 2377, 1236, 2415, 71, 1168, 821, 346, 1308, 556, 87, 1438, 862, 2344, 1380, 1909, 1131, 1765, 1367, 655, 1970, 390, 719, 2493, 1482, 1889, 542, 1764, 476, 1246, 397, 1961, 1125, 1594, 1391, 2210, 1948, 137, 2037, 143, 2167, 1360, 1452, 1310, 1411, 2414, 901, 2287, 2108, 1231, 1216, 307, 1467, 1083, 2044, 1032, 789, 169, 2106, 1483, 2253, 2466, 2330, 1567, 2263, 58, 194, 224, 1911, 1906, 165, 2103, 1588, 400, 1124, 1740, 2405, 1847, 2093, 2520, 722, 69, 2331, 1921, 1434, 160, 2395, 1424, 2496, 2345, 1077, 496, 2027, 234, 1219, 1404, 1880, 1085, 2273, 726, 313, 1186, 453, 829, 1749, 646, 2463, 2129, 1929, 2036, 1096, 461, 2299, 1387, 1066, 2531, 170, 1157, 1806, 1783, 2227, 2138, 314, 1294, 1444, 1378, 488, 643, 349, 2398, 1569, 1852, 2182, 765, 2199, 1074, 2326, 356, 1200, 1830, 1623, 2266, 2379, 2469, 2109, 2214, 1154, 1523, 1944, 2577, 973, 2342, 550, 196, 1075, 309, 2435, 1893, 1663, 2504, 2448, 1341, 585, 364, 2354, 836, 1472, 1639, 825, 1754, 842, 1439, 786, 752, 246, 1152, 615, 963, 2162, 1769, 1196, 1733, 888, 1143, 2308, 149, 2189, 464, 208, 1476, 1781, 1809, 100, 2137, 378, 1376, 466, 2098, 728, 332, 2116, 847, 1399, 1345, 1679, 2048, 463, 2507, 2339, 833, 2473, 548, 301, 2351, 2476, 1731, 7, 1899, 2250, 2392, 2426, 569, 800, 41, 24, 1763, 104, 807, 2371, 2309, 120, 1239, 2535, 2133, 620, 1343, 1729, 388, 1187, 1232, 1709, 2131, 632, 843, 1323, 561, 861, 1395, 294, 2477, 59, 2051, 363, 131, 1741, 225, 1823, 2061, 1207, 498, 2537, 718, 2464, 319, 1751, 955, 138, 2540, 1834, 1144, 1792, 1757, 72, 1858, 1170, 2073, 698, 1241, 467, 1226, 214, 1797, 1311, 2300, 1087, 1461, 1290, 2110, 1339, 1581, 1821, 478, 2543, 2384, 730, 2208, 634, 1240, 2509, 2090, 166, 2424, 1402, 1577, 778, 2, 764, 2372, 1406, 80, 1753, 121, 2566, 1136, 1939, 84, 840, 1098, 662, 352, 2296, 1782, 674, 2288, 2099, 2275, 78, 1916, 555, 583, 1073, 175, 906, 1882, 1307, 1974, 681, 794, 446, 101, 1486, 808, 2498, 1838, 1715, 217, 1890, 57, 2177, 140, 374, 1691, 1331, 2487, 1195, 540, 110, 1942, 1145, 2363, 492, 1535, 102, 1299, 1513, 744, 1456, 2484, 972, 1865, 721, 2549, 1850, 2338, 2324, 853, 1446, 1987, 1325, 2489, 1591, 1288, 477, 322, 2526, 1379, 385, 330, 1305, 2171, 947, 1317, 1976, 2298, 207, 1953, 499, 968, 2206, 2014, 757, 1908, 656, 1451, 2031, 42, 2091, 1555, 1822, 1779, 623, 1828, 1237, 1884, 243, 2427, 2113, 1985, 1931, 1095, 1841, 2053, 1068, 2039, 134, 1808, 253, 850, 2357, 2394, 1687, 2417, 379, 1481, 383, 2505, 1582, 1295, 2248, 1742, 1097, 1579, 1637, 1681, 1885, 1326, 435, 1422, 320, 2041, 1080, 146, 2486, 2314, 178, 289, 2557, 2072, 1134, 1842, 1148, 2434, 1580, 2404, 1879, 2077, 2401, 433, 218, 429, 19, 2225, 2382, 2512, 1221, 830, 1793, 1796, 539, 2168, 181, 176, 922, 2365, 296, 1928, 1920, 1027, 1756, 1261, 554, 1788, 1135, 688, 2052, 1468, 286, 254, 1463, 1372, 1589, 2254, 813, 1584, 1269, 2471, 2071, 884, 1655, 256, 2461, 1815, 2083, 2255, 55, 2529, 144, 1394, 255, 1871, 1887, 1082, 1575, 2160, 1517, 890, 14, 128, 1734, 1362, 2572, 2456, 338, 2056, 1183, 869, 748, 1322, 473, 2002, 1089, 762, 1914, 495, 9, 380, 203, 2374, 28, 2409, 1773, 306, 241, 2437, 440, 1172, 1673, 2144, 1250, 2201, 115, 2203, 454, 724, 2001, 2538, 155, 2096, 1878, 1245, 2488, 2038, 1598, 1606, 1203, 952, 1837, 1876, 1396, 839, 1563, 1419, 2204, 2534, 1298, 26, 111, 1836, 162, 907, 874, 675, 1272, 262, 1759, 1164, 1965, 1979, 2134, 272, 2446, 1197, 1464, 2148, 250, 1772, 1266, 1972, 97, 2127, 2411, 1353, 298, 867, 860, 1738, 1730, 2500, 1329, 1897, 815, 797, 133, 1417, 284, 1471, 912, 713, 1383, 396, 2222, 806, 2483, 2381, 227, 344, 911, 2055, 357, 1442, 740, 1015, 6, 1336, 1573, 1531, 2356, 206, 549, 1128, 2261, 1182, 2570, 2551, 2212, 474, 817, 2519, 614, 2468, 1748, 34, 184, 293, 1553, 159, 974, 1487, 2388, 1409, 1090, 491, 784, 2503, 552, 2180, 1365, 365, 1543, 1605, 2361, 1100, 1496, 617, 746, 852, 216, 1643, 2244, 2364, 2547, 1840, 1527, 300, 2310, 2295, 1369, 816, 1833, 173, 2390, 99, 125, 704, 1359, 2491, 1304, 1230, 702, 2443, 1984, 1318, 1309, 12, 609, 2257, 1238, 2428, 264, 2174, 910, 443, 1448, 347, 1174, 327, 2132, 1811, 2333, 1408, 1405, 251, 736, 1093, 186, 317, 1223, 1130, 1957, 1159, 36, 1450, 676, 68, 358, 1817, 2062, 917, 1547, 1952, 1303, 2521, 2058, 2276, 1969, 965, 1671, 755, 1324, 1358, 1158, 2230, 1132, 2190, 1991, 960, 2272, 2376, 1333, 723, 153, 1780, 2231, 2262, 1835, 2327, 2164, 541, 260, 582, 1079, 1184, 2247, 1285, 2112, 2335, 209, 2550, 627, 647, 2057, 2270, 2120, 10, 156, 394, 1070, 1886, 976, 1366, 660, 1766, 142, 1923, 2179, 93, 1950, 2399, 1264, 2035, 86, 1401, 436, 399, 727, 1724, 484, 1819, 657, 324, 1274, 1967, 2369, 1115, 1208, 1762, 1163, 1609, 1998, 961, 1874, 1647, 1385, 822, 2480, 369, 1945, 1907, 2021, 2170, 1982, 967, 1768, 2032, 566, 2452, 1171, 304, 2078, 2391, 481, 122, 2429, 2006, 2150, 1938, 913, 305, 1515, 1786, 559, 2010, 1121, 1977, 2373, 278, 191, 611, 1597, 567, 1760, 2341, 2197, 2193, 1994, 2560, 1917, 2347, 1058, 1747, 2544, 1625, 2343, 1447, 1995, 2054, 432, 624, 1675, 1604, 1206, 810, 3, 29, 1381, 1332, 1228, 1707, 2094, 2269, 2265, 1165, 1927, 2220, 714, 1721, 1347, 2455, 959, 2155, 1477, 2454, 2558, 2143, 33, 1210, 1357, 1910, 1342, 441, 2280, 82, 1225, 2159, 132, 1963, 2278, 2286, 1213, 682, 469, 2018, 1262, 1218, 15, 38, 493, 1846, 1386, 775, 2092, 238, 1649, 2481, 1683, 1327, 1478, 2539, 2022, 1933, 1665, 21, 610, 886, 896, 1312, 288, 108, 2340, 2082, 1398, 2011, 1930, 1092, 1485, 2475, 756, 2317, 690, 220, 1964, 1224, 2045, 2119, 2436, 43, 787, 2023, 2252, 2472, 2017, 213, 1744, 855, 1489, 1233, 1925, 579, 1415, 2223, 2536, 908, 1723, 395, 1798, 2541, 1390, 2237, 588, 2532, 2260, 1102, 350, 94, 342, 1746, 1771, 1235, 239, 826, 1363, 1863, 2459, 360, 1212, 763, 2561, 1497, 2291, 1946, 1257, 2097, 2294, 2389, 113, 1565, 716, 2439, 1602, 345, 2306, 386, 1111, 2198, 2013, 318, 1954, 1631, 1110, 158, 163, 1198, 773, 0, 2030, 1169, 2196, 1149, 1133, 2191, 449, 760, 96, 2329, 45, 1107, 1441, 123, 882, 2183, 2568, 534, 1895, 1105, 557, 1160, 1973, 1426, 172, 1794, 1803, 2297, 2495, 39, 2122, 2256, 1915, 2136, 1571, 1820, 1525, 2207, 261, 66, 1204, 257, 1243, 1418, 202, 368, 680, 1968, 1785, 2118, 1812, 221, 524, 2444, 1178, 1728, 805, 348, 291, 1719, 848, 76, 673, 1935, 1755, 462, 370, 1024, 1922, 1189, 1277, 2000, 2070, 1849, 1440, 2249, 1586, 222, 1449, 659, 1101, 2175, 2228, 1088, 1321, 1457, 2215, 1061, 1941, 1352, 2232, 2064, 2142, 1397, 1949, 1891, 1334, 438, 2431, 1466, 2046, 457, 1600, 2383, 1645, 152, 946, 2358, 1407, 1430, 1677, 393, 460, 2101, 1455, 918, 1501, 962, 211, 1423, 258, 46, 2279, 2243, 1181, 1900, 302, 195, 1023, 392, 558, 2229, 1993, 27, 377, 1361, 2511, 2449, 792, 1633, 237, 1503, 1499, 1585, 831, 1997, 2095, 325, 1273, 1859, 2005, 2187, 1113, 1537, 2569, 297, 1981, 547, 536, 2360, 1142, 2311, 622, 109, 2393, 2012, 1713, 804, 1156, 546, 1592, 2458, 75, 2385, 809, 2029, 838, 1986, 785, 361, 608, 89, 589, 1429, 2139, 231, 1905, 2346, 1433, 1697, 1374, 259, 479, 276, 2158, 948, 268, 920, 1870, 2050, 1248, 2573, 2325, 1393, 2043, 201, 2406, 1826, 63, 2277, 2245, 2413, 2188, 1215, 772, 2362, 1689, 921, 1063, 1375, 1091, 1120, 1022, 2578, 2066, 2178, 544, 236, 2559, 124, 486, 1864, 2430, 834, 452, 1026, 2026, 923, 114, 341, 1829, 782, 1104, 1490, 2523, 712, 2422, 482, 1175, 2563, 1436, 767, 1256, 1416, 1888, 1784, 2478, 2281, 391, 303, 1109, 372, 2258, 1059, 2146, 779, 1030, 1505, 489, 909, 633, 167, 1081, 1902, 139, 618, 145, 2328, 232, 2506, 2508, 2200, 32, 1431, 430, 2194, 894, 351, 77, 1123, 1661, 1736, 2567, 1620, 2081, 2451, 1179, 1029, 1147, 1621, 1382, 2135, 1199, 1711, 2219, 2234, 353, 2145, 586, 1549, 81, 1031, 367, 537, 1750, 2025, 157, 1495, 472, 1659, 1071, 79, 1699, 1190, 1988, 2104, 2216, 798, 2497, 1214, 2125, 1956, 1824, 1831, 1328, 1732, 177, 371, 497, 129, 16, 2195, 975, 54, 2040, 2524, 1903, 2334, 725, 1289, 229, 1078, 1354, 2003, 1258, 1084, 1116, 1188, 2233, 580, 1117, 553, 878, 2370, 1388, 1989, 23, 1491, 1883, 2016, 1191, 1283, 389, 1229, 2107, 2320, 563, 2202, 329, 315, 684, 1351, 1138, 366, 328, 1845, 2412, 2301, 2375, 1774, 2292, 1657, 1519, 1971, 40, 1356, 252, 854, 2402, 1086, 2318, 1253, 1458, 803, 1958, 233, 359, 1578, 1545, 2049, 761, 1894, 1717, 1601, 696, 1875, 189, 1726, 47, 2047, 2007, 1192, 914, 1839, 677, 1778, 2441, 64, 1825, 2126, 1392, 2085, 2089, 2128, 977, 2074, 700, 1316, 51, 398, 1146, 2457, 373, 1955, 1868, 1752, 612, 67, 2019, 20, 1990, 384, 1177, 1758, 2423, 1118, 2111, 1480, 105, 956, 248, 710, 2181, 22, 1866, 1400, 2321, 2086, 2438, 2271, 898, 1453, 776, 2479, 958, 2378, 1653, 812, 863, 616, 2522, 694, 642, 1862, 2242, 1389, 1892, 774, 2154, 2465, 2307, 2420, 343, 228, 287, 1094, 1279, 53, 1791, 1454, 1276, 1790, 835, 91, 2450, 442, 1425, 678, 2238, 13, 269, 876, 1583, 439, 2024, 820, 799, 543, 1296, 1114, 745, 2130, 2087, 1599, 2460, 2008, 1912, 1940, 2575, 1492, 1162, 790, 819, 2105, 1775, 717, 953, 916, 2337, 1813, 428, 1126, 2445, 841, 212, 783, 2123, 729, 1473, 846, 215, 2209, 354, 274, 127, 2380, 796, 551, 198, 2289, 759, 1848, 851, 1999, 905, 1062, 2352, 868, 2513, 1153, 2185, 471, 1428, 1978, 1610, 1521, 190, 1268, 1099, 2240, 1913, 2518, 1992, 154, 1725, 1234, 308, 147, 1624, 56, 1150, 205, 1384, 1494, 2205, 290, 331, 1291, 2284, 2350, 266, 62, 971, 1622, 2302, 1265, 199, 70, 1924, 1350, 136, 1901, 641, 270, 263, 376, 587, 242, 2442, 1818, 648, 2515, 1185, 180, 1507, 2571, 2304, 2546, 1445, 1860, 25, 966, 1344, 1278, 168, 738, 2499, 1805, 856, 2080, 31, 2407, 715, 1693, 2490, 2009, 193, 1533, 141, 2494, 1161, 1854, 1202, 2467, 2528, 1137, 2548, 2312, 1867, 1611, 2564, 1488, 2332, 2397, 197, 2065, 2063, 849, 1801, 267, 1112, 1370, 1259, 1403, 1896, 538, 382, 1877, 1413, 2462, 326, 621, 447, 1271, 903, 2433, 1484, 1222, 375, 1255, 1348, 1509, 1576, 1743, 2069, 2574, 2285, 1414, 311, 2349, 2403, 613, 2576, 1242, 2268, 795, 1804, 247, 148, 1313, 758, 870, 1844, 2173, 455, 2151, 970, 2152, 2224, 2147, 85, 1947, 174, 1613, 1437, 192, 832, 1193, 2165, 323, 1284, 490, 957, 1511, 1475, 35, 1067, 1432, 116, 2236, 845, 1108, 50, 644, 1462, 2517, 2282, 2251, 1745, 2474, 590, 2533, 285, 802, 2157, 1596, 2192, 1587, 1205, 1651, 1459, 1810, 2274, 401, 1346, 788, 1904, 1807, 450, 321, 768, 1338, 2516, 204, 1300, 564, 2552, 1843, 179, 658, 2556, 1983, 1421, 2419, 1561, 2102, 1025, 1787, 2172, 1767, 1607, 1737, 1960, 2387, 1857, 2303, 226, 1076, 2226, 1028, 1315, 2076, 1898, 1377, 2453, 2235, 1460, 950, 741, 2514, 2141, 686, 915, 2565, 2088, 1167, 844, 4, 118, 1420, 112, 1201, 2501, 164, 2527, 2447, 187, 2176, 244, 969, 2348, 2015, 103, 494, 1443, 1943, 1541, 1254, 339, 60, 312, 444, 892, 1209, 2492, 2264, 2502, 92, 458, 1065, 1371, 2117, 265, 49, 1635, 1286, 1292, 135, 565, 1064, 2367, 1021, 1103, 171, 739, 1739, 2482, 2485, 754, 210, 1869, 1735, 1368, 316, 1761, 2161, 2084, 1926, 2068, 1270, 1727, 2028, 2213, 1937, 1355, 183, 1127, 1919, 182, 434, 1410, 1180, 1247, 2315, 2386, 1, 2079, 1469, 1069, 2042, 1685, 1249, 161, 1557, 470, 1872, 880, 742, 777, 630, 2530, 2316, 2259, 1020, 1776, 2184, 545, 1603, 387, 126, 2067, 2020, 919, 1337, 1072, 465, 200, 1319, 865, 292, 5, 1435, 766, 562, 2293, 1263, 951, 117, 235, 2440, 485, 17, 1770, 2075, 2169, 2368, 1590, 720, 240, 535, 451, 431, 2410, 448, 2425, 475, 37, 2033, 52, 2241, 964, 2100, 2323, 2408, 1814, 487, 230, 837, 2510, 2290, 1330, 2034, 2555, 48, 900, 1802, 679, 1287, 949, 1302, 1227, 1139, 30, 591, 2217, 18, 2153, 483, 1795, 866, 1932, 1260, 584, 2186, 74, 1881, 751, 2060, 2336, 1373, 1267, 801, 1220, 857, 1705, 1251, 1832, 2267, 1856, 2400, 1122, 1282, 1934, 151, 1340, 188, 692, 44, 2140, 1217, 2059, 1641, 631, 1297, 2221, 2004, 1479, 1176, 1966, 2121, 282, 1539, 340, 818, 814, 1851, 2211, 747, 793, 2416, 1595, 1695, 355, 1320, 2115, 362, 753, 2418, 828, 1777, 1119, 2432, 119, 1855, 2239, 1293, 150, 1799, 2525, 619, 468, 8, 1996, 708, 1816, 791, 2421, 1140, 1975, 2353, 706, 1980, 2246, 1608, 1703, 1962, 456, 2149, 480, 90, 645, 581, 1314, 2553, 1166, 1669, 2359, 902, 1667, 106, 743, 2305, 1194, 1335, 2283, 1612, 1244, 1873],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_UPPER_KEYSYM: IndexMap<u32, KeysymCaseMapping> = IndexMap {
    key: 12913932095322966823,
    disps: &[(0, 12), (0, 0), (0, 10), (0, 5), (0, 26), (0, 20), (0, 8), (0, 0), (0, 4), (0, 0), (0, 18), (0, 35), (0, 1), (0, 0), (0, 0), (0, 52), (0, 5), (0, 4), (0, 26), (0, 1), (0, 3), (0, 0), (0, 2), (0, 1), (0, 2), (0, 2), (0, 9), (0, 0), (0, 0), (0, 8), (0, 0), (0, 55), (0, 3), (0, 1), (0, 1), (0, 3), (0, 0), (0, 18), (0, 2), (0, 124), (0, 0), (0, 11), (0, 164), (0, 10), (0, 2), (0, 85), (2, 10), (0, 56), (0, 67), (0, 36), (1, 34), (0, 0), (0, 100), (0, 23), (0, 64), (0, 95), (0, 7), (0, 5), (0, 144), (0, 0), (1, 65), (0, 50), (0, 1), (0, 0)],
    map: &[KeysymCaseMapping { keysym: 0x06db, other: 0x000006fb }, KeysymCaseMapping { keysym: 0x06cc, other: 0x000006ec }, KeysymCaseMapping { keysym: 0x07f4, other: 0x000007d4 }, KeysymCaseMapping { keysym: 0x07b1, other: 0x000007a1 }, KeysymCaseMapping { keysym: 0x00f4, other: 0x000000d4 }, KeysymCaseMapping { keysym: 0x07e4, other: 0x000007c4 }, KeysymCaseMapping { keysym: 0x07e2, other: 0x000007c2 }, KeysymCaseMapping { keysym: 0x01b6, other: 0x000001a6 }, KeysymCaseMapping { keysym: 0x00ec, other: 0x000000cc }, KeysymCaseMapping { keysym: 0x03b5, other: 0x000003a5 }, KeysymCaseMapping { keysym: 0x01f5, other: 0x000001d5 }, KeysymCaseMapping { keysym: 0x06aa, other: 0x000006ba }, KeysymCaseMapping { keysym: 0x03e0, other: 0x000003c0 }, KeysymCaseMapping { keysym: 0x01fe, other: 0x000001de }, KeysymCaseMapping { keysym: 0x07e8, other: 0x000007c8 }, KeysymCaseMapping { keysym: 0x07ed, other: 0x000007cd }, KeysymCaseMapping { keysym: 0x06a6, other: 0x000006b6 }, KeysymCaseMapping { keysym: 0x01ea, other: 0x000001ca }, KeysymCaseMapping { keysym: 0x07f7, other: 0x000007d7 }, KeysymCaseMapping { keysym: 0x02b1, other: 0x000002a1 }, KeysymCaseMapping { keysym: 0x00e9, other: 0x000000c9 }, KeysymCaseMapping { keysym: 0x01e6, other: 0x000001c6 }, KeysymCaseMapping { keysym: 0x06c1, other: 0x000006e1 }, KeysymCaseMapping { keysym: 0x01b9, other: 0x000001a9 }, KeysymCaseMapping { keysym: 0x01fb, other: 0x000001db }, KeysymCaseMapping { keysym: 0x00f6, other: 0x000000d6 }, KeysymCaseMapping { keysym: 0x06d2, other: 0x000006f2 }, KeysymCaseMapping { keysym: 0x00ee, other: 0x000000ce }, KeysymCaseMapping { keysym: 0x03ec, other: 0x000003cc }, KeysymCaseMapping { keysym: 0x01b5, other: 0x000001a5 }, KeysymCaseMapping { keysym: 0x07f9, other: 0x000007d9 }, KeysymCaseMapping { keysym: 0x01f9, other: 0x000001d9 }, KeysymCaseMapping { keysym: 0x01ef, other: 0x000001cf }, KeysymCaseMapping { keysym: 0x00e5, other: 0x000000c5 }, KeysymCaseMapping { keysym: 0x06ce, other: 0x000006ee }, KeysymCaseMapping { keysym: 0x07ea, other: 0x000007ca }, KeysymCaseMapping { keysym: 0x02b6, other: 0x000002a6 }, KeysymCaseMapping { keysym: 0x01f8, other: 0x000001d8 }, KeysymCaseMapping { keysym: 0x07b9, other: 0x000007a9 }, KeysymCaseMapping { keysym: 0x02f8, other: 0x000002d8 }, KeysymCaseMapping { keysym: 0x06df, other: 0x000006ff }, KeysymCaseMapping { keysym: 0x00fd, other: 0x000000dd }, KeysymCaseMapping { keysym: 0x00eb, other: 0x000000cb }, KeysymCaseMapping { keysym: 0x00ff, other: 0x000013be }, KeysymCaseMapping { keysym: 0x00e7, other: 0x000000c7 }, KeysymCaseMapping { keysym: 0x01f0, other: 0x000001d0 }, KeysymCaseMapping { keysym: 0x03fd, other: 0x000003dd }, KeysymCaseMapping { keysym: 0x00fc, other: 0x000000dc }, KeysymCaseMapping { keysym: 0x07b5, other: 0x000007a5 }, KeysymCaseMapping { keysym: 0x06d8, other: 0x000006f8 }, KeysymCaseMapping { keysym: 0x00f3, other: 0x000000d3 }, KeysymCaseMapping { keysym: 0x06c3, other: 0x000006e3 }, KeysymCaseMapping { keysym: 0x07ef, other: 0x000007cf }, KeysymCaseMapping { keysym: 0x07f3, other: 0x000007d2 }, KeysymCaseMapping { keysym: 0x06d4, other: 0x000006f4 }, KeysymCaseMapping { keysym: 0x06a1, other: 0x000006b1 }, KeysymCaseMapping { keysym: 0x06a5, other: 0x000006b5 }, KeysymCaseMapping { keysym: 0x07f0, other: 0x000007d0 }, KeysymCaseMapping { keysym: 0x06a2, other: 0x000006b2 }, KeysymCaseMapping { keysym: 0x06c0, other: 0x000006e0 }, KeysymCaseMapping { keysym: 0x06a9, other: 0x000006b9 }, KeysymCaseMapping { keysym: 0x01b3, other: 0x000001a3 }, KeysymCaseMapping { keysym: 0x07e3, other: 0x000007c3 }, KeysymCaseMapping { keysym: 0x02bc, other: 0x000002ac }, KeysymCaseMapping { keysym: 0x07ec, other: 0x000007cc }, KeysymCaseMapping { keysym: 0x06dd, other: 0x000006fd }, KeysymCaseMapping { keysym: 0x07b3, other: 0x000007a3 }, KeysymCaseMapping { keysym: 0x07bb, other: 0x000007ab }, KeysymCaseMapping { keysym: 0x08f6, other: 0x01000191 }, KeysymCaseMapping { keysym: 0x00ed, other: 0x000000cd }, KeysymCaseMapping { keysym: 0x06ae, other: 0x000006be }, KeysymCaseMapping { keysym: 0x01f2, other: 0x000001d2 }, KeysymCaseMapping { keysym: 0x03bb, other: 0x000003ab }, KeysymCaseMapping { keysym: 0x07b7, other: 0x000007a7 }, KeysymCaseMapping { keysym: 0x02b9, other: 0x00000049 }, KeysymCaseMapping { keysym: 0x03e7, other: 0x000003c7 }, KeysymCaseMapping { keysym: 0x03ba, other: 0x000003aa }, KeysymCaseMapping { keysym: 0x06d1, other: 0x000006f1 }, KeysymCaseMapping { keysym: 0x06c5, other: 0x000006e5 }, KeysymCaseMapping { keysym: 0x07e1, other: 0x000007c1 }, KeysymCaseMapping { keysym: 0x03b6, other: 0x000003a6 }, KeysymCaseMapping { keysym: 0x06c9, other: 0x000006e9 }, KeysymCaseMapping { keysym: 0x07f1, other: 0x000007d1 }, KeysymCaseMapping { keysym: 0x06ab, other: 0x000006bb }, KeysymCaseMapping { keysym: 0x00fe, other: 0x000000de }, KeysymCaseMapping { keysym: 0x06a3, other: 0x000006b3 }, KeysymCaseMapping { keysym: 0x00e2, other: 0x000000c2 }, KeysymCaseMapping { keysym: 0x07b2, other: 0x000007a2 }, KeysymCaseMapping { keysym: 0x06c2, other: 0x000006e2 }, KeysymCaseMapping { keysym: 0x00f1, other: 0x000000d1 }, KeysymCaseMapping { keysym: 0x01e3, other: 0x000001c3 }, KeysymCaseMapping { keysym: 0x07ee, other: 0x000007ce }, KeysymCaseMapping { keysym: 0x01ba, other: 0x000001aa }, KeysymCaseMapping { keysym: 0x06d6, other: 0x000006f6 }, KeysymCaseMapping { keysym: 0x03f1, other: 0x000003d1 }, KeysymCaseMapping { keysym: 0x01ec, other: 0x000001cc }, KeysymCaseMapping { keysym: 0x07e6, other: 0x000007c6 }, KeysymCaseMapping { keysym: 0x00ef, other: 0x000000cf }, KeysymCaseMapping { keysym: 0x06d3, other: 0x000006f3 }, KeysymCaseMapping { keysym: 0x07f5, other: 0x000007d5 }, KeysymCaseMapping { keysym: 0x06cb, other: 0x000006eb }, KeysymCaseMapping { keysym: 0x02bb, other: 0x000002ab }, KeysymCaseMapping { keysym: 0x01bb, other: 0x000001ab }, KeysymCaseMapping { keysym: 0x02fd, other: 0x000002dd }, KeysymCaseMapping { keysym: 0x06c7, other: 0x000006e7 }, KeysymCaseMapping { keysym: 0x02f5, other: 0x000002d5 }, KeysymCaseMapping { keysym: 0x07f2, other: 0x000007d2 }, KeysymCaseMapping { keysym: 0x06d0, other: 0x000006f0 }, KeysymCaseMapping { keysym: 0x01e0, other: 0x000001c0 }, KeysymCaseMapping { keysym: 0x03bf, other: 0x000003bd }, KeysymCaseMapping { keysym: 0x00e8, other: 0x000000c8 }, KeysymCaseMapping { keysym: 0x01bf, other: 0x000001af }, KeysymCaseMapping { keysym: 0x06ad, other: 0x000006bd }, KeysymCaseMapping { keysym: 0x01f1, other: 0x000001d1 }, KeysymCaseMapping { keysym: 0x06da, other: 0x000006fa }, KeysymCaseMapping { keysym: 0x06c8, other: 0x000006e8 }, KeysymCaseMapping { keysym: 0x06dc, other: 0x000006fc }, KeysymCaseMapping { keysym: 0x06c4, other: 0x000006e4 }, KeysymCaseMapping { keysym: 0x00e4, other: 0x000000c4 }, KeysymCaseMapping { keysym: 0x03fe, other: 0x000003de }, KeysymCaseMapping { keysym: 0x06d9, other: 0x000006f9 }, KeysymCaseMapping { keysym: 0x01bc, other: 0x000001ac }, KeysymCaseMapping { keysym: 0x00f5, other: 0x000000d5 }, KeysymCaseMapping { keysym: 0x03f3, other: 0x000003d3 }, KeysymCaseMapping { keysym: 0x00f9, other: 0x000000d9 }, KeysymCaseMapping { keysym: 0x00e0, other: 0x000000c0 }, KeysymCaseMapping { keysym: 0x01e5, other: 0x000001c5 }, KeysymCaseMapping { keysym: 0x06d5, other: 0x000006f5 }, KeysymCaseMapping { keysym: 0x02e6, other: 0x000002c6 }, KeysymCaseMapping { keysym: 0x03ef, other: 0x000003cf }, KeysymCaseMapping { keysym: 0x00e1, other: 0x000000c1 }, KeysymCaseMapping { keysym: 0x07b8, other: 0x000007a8 }, KeysymCaseMapping { keysym: 0x07e9, other: 0x000007c9 }, KeysymCaseMapping { keysym: 0x07f8, other: 0x000007d8 }, KeysymCaseMapping { keysym: 0x06cd, other: 0x000006ed }, KeysymCaseMapping { keysym: 0x06ac, other: 0x000006bc }, KeysymCaseMapping { keysym: 0x00f2, other: 0x000000d2 }, KeysymCaseMapping { keysym: 0x00ea, other: 0x000000ca }, KeysymCaseMapping { keysym: 0x01b1, other: 0x000001a1 }, KeysymCaseMapping { keysym: 0x03bc, other: 0x000003ac }, KeysymCaseMapping { keysym: 0x07e5, other: 0x000007c5 }, KeysymCaseMapping { keysym: 0x07b4, other: 0x000007a4 }, KeysymCaseMapping { keysym: 0x06ca, other: 0x000006ea }, KeysymCaseMapping { keysym: 0x06a7, other: 0x000006b7 }, KeysymCaseMapping { keysym: 0x02fe, other: 0x000002de }, KeysymCaseMapping { keysym: 0x00fb, other: 0x000000db }, KeysymCaseMapping { keysym: 0x07f6, other: 0x000007d6 }, KeysymCaseMapping { keysym: 0x00e6, other: 0x000000c6 }, KeysymCaseMapping { keysym: 0x00b5, other: 0x000007cc }, KeysymCaseMapping { keysym: 0x03f9, other: 0x000003d9 }, KeysymCaseMapping { keysym: 0x01be, other: 0x000001ae }, KeysymCaseMapping { keysym: 0x06a8, other: 0x000006b8 }, KeysymCaseMapping { keysym: 0x13bd, other: 0x000013bc }, KeysymCaseMapping { keysym: 0x06c6, other: 0x000006e6 }, KeysymCaseMapping { keysym: 0x06d7, other: 0x000006f7 }, KeysymCaseMapping { keysym: 0x00e3, other: 0x000000c3 }, KeysymCaseMapping { keysym: 0x00fa, other: 0x000000da }, KeysymCaseMapping { keysym: 0x03b3, other: 0x000003a3 }, KeysymCaseMapping { keysym: 0x00f8, other: 0x000000d8 }, KeysymCaseMapping { keysym: 0x06cf, other: 0x000006ef }, KeysymCaseMapping { keysym: 0x07eb, other: 0x000007cb }, KeysymCaseMapping { keysym: 0x00f0, other: 0x000000d0 }, KeysymCaseMapping { keysym: 0x06de, other: 0x000006fe }, KeysymCaseMapping { keysym: 0x06af, other: 0x000006bf }, KeysymCaseMapping { keysym: 0x06a4, other: 0x000006b4 }, KeysymCaseMapping { keysym: 0x07e7, other: 0x000007c7 }, KeysymCaseMapping { keysym: 0x03f2, other: 0x000003d2 }, KeysymCaseMapping { keysym: 0x01e8, other: 0x000001c8 }, KeysymCaseMapping { keysym: 0x02e5, other: 0x000002c5 }],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_LOWER_KEYSYM: IndexMap<u32, KeysymCaseMapping> = IndexMap {
    key: 12913932095322966823,
    disps: &[(0, 1), (0, 2), (0, 0), (0, 3), (0, 21), (0, 15), (0, 8), (0, 3), (0, 0), (0, 3), (0, 3), (0, 52), (0, 76), (0, 8), (0, 16), (0, 5), (0, 0), (0, 17), (0, 0), (0, 0), (0, 8), (0, 0), (0, 0), (0, 25), (0, 71), (0, 24), (0, 3), (0, 10), (0, 4), (0, 4), (0, 2), (0, 76), (0, 0), (0, 99), (0, 1), (0, 15), (0, 37), (0, 4), (0, 6), (0, 53), (0, 0), (0, 0), (0, 5), (0, 8), (0, 37), (0, 0), (0, 48), (0, 23), (0, 6), (0, 30), (0, 3), (0, 8), (0, 42), (0, 24), (1, 16), (0, 0), (1, 62), (1, 61), (0, 17), (0, 0), (0, 66), (0, 0), (0, 2), (0, 26)],
    map: &[KeysymCaseMapping { keysym: 0x06b5, other: 0x000006a5 }, KeysymCaseMapping { keysym: 0x07d9, other: 0x000007f9 }, KeysymCaseMapping { keysym: 0x00da, other: 0x000000fa }, KeysymCaseMapping { keysym: 0x01de, other: 0x000001fe }, KeysymCaseMapping { keysym: 0x03d9, other: 0x000003f9 }, KeysymCaseMapping { keysym: 0x03ac, other: 0x000003bc }, KeysymCaseMapping { keysym: 0x06e8, other: 0x000006c8 }, KeysymCaseMapping { keysym: 0x00d2, other: 0x000000f2 }, KeysymCaseMapping { keysym: 0x01c5, other: 0x000001e5 }, KeysymCaseMapping { keysym: 0x07a8, other: 0x000007b8 }, KeysymCaseMapping { keysym: 0x07ca, other: 0x000007ea }, KeysymCaseMapping { keysym: 0x06e4, other: 0x000006c4 }, KeysymCaseMapping { keysym: 0x07d5, other: 0x000007f5 }, KeysymCaseMapping { keysym: 0x06e9, other: 0x000006c9 }, KeysymCaseMapping { keysym: 0x07a3, other: 0x000007b3 }, KeysymCaseMapping { keysym: 0x01ca, other: 0x000001ea }, KeysymCaseMapping { keysym: 0x06f9, other: 0x000006d9 }, KeysymCaseMapping { keysym: 0x06fe, other: 0x000006de }, KeysymCaseMapping { keysym: 0x01d5, other: 0x000001f5 }, KeysymCaseMapping { keysym: 0x01a3, other: 0x000001b3 }, KeysymCaseMapping { keysym: 0x00c3, other: 0x000000e3 }, KeysymCaseMapping { keysym: 0x00cd, other: 0x000000ed }, KeysymCaseMapping { keysym: 0x00c7, other: 0x000000e7 }, KeysymCaseMapping { keysym: 0x00c6, other: 0x000000e6 }, KeysymCaseMapping { keysym: 0x00dd, other: 0x000000fd }, KeysymCaseMapping { keysym: 0x00c8, other: 0x000000e8 }, KeysymCaseMapping { keysym: 0x03bd, other: 0x000003bf }, KeysymCaseMapping { keysym: 0x07d0, other: 0x000007f0 }, KeysymCaseMapping { keysym: 0x06bb, other: 0x000006ab }, KeysymCaseMapping { keysym: 0x02a6, other: 0x000002b6 }, KeysymCaseMapping { keysym: 0x02dd, other: 0x000002fd }, KeysymCaseMapping { keysym: 0x06ef, other: 0x000006cf }, KeysymCaseMapping { keysym: 0x07cb, other: 0x000007eb }, KeysymCaseMapping { keysym: 0x03a3, other: 0x000003b3 }, KeysymCaseMapping { keysym: 0x06ee, other: 0x000006ce }, KeysymCaseMapping { keysym: 0x03c0, other: 0x000003e0 }, KeysymCaseMapping { keysym: 0x01c6, other: 0x000001e6 }, KeysymCaseMapping { keysym: 0x01d0, other: 0x000001f0 }, KeysymCaseMapping { keysym: 0x00d3, other: 0x000000f3 }, KeysymCaseMapping { keysym: 0x06f4, other: 0x000006d4 }, KeysymCaseMapping { keysym: 0x02ab, other: 0x000002bb }, KeysymCaseMapping { keysym: 0x07d6, other: 0x000007f6 }, KeysymCaseMapping { keysym: 0x06b1, other: 0x000006a1 }, KeysymCaseMapping { keysym: 0x01c0, other: 0x000001e0 }, KeysymCaseMapping { keysym: 0x06e5, other: 0x000006c5 }, KeysymCaseMapping { keysym: 0x01ae, other: 0x000001be }, KeysymCaseMapping { keysym: 0x06b6, other: 0x000006a6 }, KeysymCaseMapping { keysym: 0x13be, other: 0x000000ff }, KeysymCaseMapping { keysym: 0x02a1, other: 0x000002b1 }, KeysymCaseMapping { keysym: 0x02d8, other: 0x000002f8 }, KeysymCaseMapping { keysym: 0x06ff, other: 0x000006df }, KeysymCaseMapping { keysym: 0x01db, other: 0x000001fb }, KeysymCaseMapping { keysym: 0x00c9, other: 0x000000e9 }, KeysymCaseMapping { keysym: 0x07cc, other: 0x000007ec }, KeysymCaseMapping { keysym: 0x07c7, other: 0x000007e7 }, KeysymCaseMapping { keysym: 0x07c1, other: 0x000007e1 }, KeysymCaseMapping { keysym: 0x01a9, other: 0x000001b9 }, KeysymCaseMapping { keysym: 0x06e0, other: 0x000006c0 }, KeysymCaseMapping { keysym: 0x07d1, other: 0x000007f1 }, KeysymCaseMapping { keysym: 0x00de, other: 0x000000fe }, KeysymCaseMapping { keysym: 0x06bc, other: 0x000006ac }, KeysymCaseMapping { keysym: 0x06b7, other: 0x000006a7 }, KeysymCaseMapping { keysym: 0x06f5, other: 0x000006d5 }, KeysymCaseMapping { keysym: 0x02de, other: 0x000002fe }, KeysymCaseMapping { keysym: 0x01d1, other: 0x000001f1 }, KeysymCaseMapping { keysym: 0x00d4, other: 0x000000f4 }, KeysymCaseMapping { keysym: 0x01cc, other: 0x000001ec }, KeysymCaseMapping { keysym: 0x07a4, other: 0x000007b4 }, KeysymCaseMapping { keysym: 0x01af, other: 0x000001bf }, KeysymCaseMapping { keysym: 0x00c2, other: 0x000000e2 }, KeysymCaseMapping { keysym: 0x06eb, other: 0x000006cb }, KeysymCaseMapping { keysym: 0x06e6, other: 0x000006c6 }, KeysymCaseMapping { keysym: 0x07d7, other: 0x000007f7 }, KeysymCaseMapping { keysym: 0x00d9, other: 0x000000f9 }, KeysymCaseMapping { keysym: 0x00cf, other: 0x000000ef }, KeysymCaseMapping { keysym: 0x06f0, other: 0x000006d0 }, KeysymCaseMapping { keysym: 0x02ac, other: 0x000002bc }, KeysymCaseMapping { keysym: 0x00c4, other: 0x000000e4 }, KeysymCaseMapping { keysym: 0x07a5, other: 0x000007b5 }, KeysymCaseMapping { keysym: 0x07c6, other: 0x000007e6 }, KeysymCaseMapping { keysym: 0x03cc, other: 0x000003ec }, KeysymCaseMapping { keysym: 0x01aa, other: 0x000001ba }, KeysymCaseMapping { keysym: 0x01a5, other: 0x000001b5 }, KeysymCaseMapping { keysym: 0x00c5, other: 0x000000e5 }, KeysymCaseMapping { keysym: 0x07ab, other: 0x000007bb }, KeysymCaseMapping { keysym: 0x00d8, other: 0x000000f8 }, KeysymCaseMapping { keysym: 0x07cd, other: 0x000007ed }, KeysymCaseMapping { keysym: 0x13bc, other: 0x000013bd }, KeysymCaseMapping { keysym: 0x06fb, other: 0x000006db }, KeysymCaseMapping { keysym: 0x00ca, other: 0x000000ea }, KeysymCaseMapping { keysym: 0x06bd, other: 0x000006ad }, KeysymCaseMapping { keysym: 0x01cf, other: 0x000001ef }, KeysymCaseMapping { keysym: 0x07c2, other: 0x000007e2 }, KeysymCaseMapping { keysym: 0x07d2, other: 0x000007f2 }, KeysymCaseMapping { keysym: 0x06f8, other: 0x000006d8 }, KeysymCaseMapping { keysym: 0x06e1, other: 0x000006c1 }, KeysymCaseMapping { keysym: 0x03a5, other: 0x000003b5 }, KeysymCaseMapping { keysym: 0x01d2, other: 0x000001f2 }, KeysymCaseMapping { keysym: 0x00d5, other: 0x000000f5 }, KeysymCaseMapping { keysym: 0x07c8, other: 0x000007e8 }, KeysymCaseMapping { keysym: 0x00c0, other: 0x000000e0 }, KeysymCaseMapping { keysym: 0x06b8, other: 0x000006a8 }, KeysymCaseMapping { keysym: 0x06b3, other: 0x000006a3 }, KeysymCaseMapping { keysym: 0x06ec, other: 0x000006cc }, KeysymCaseMapping { keysym: 0x06f6, other: 0x000006d6 }, KeysymCaseMapping { keysym: 0x03d2, other: 0x000003f2 }, KeysymCaseMapping { keysym: 0x06f1, other: 0x000006d1 }, KeysymCaseMapping { keysym: 0x06fa, other: 0x000006da }, KeysymCaseMapping { keysym: 0x07c3, other: 0x000007e3 }, KeysymCaseMapping { keysym: 0x01a6, other: 0x000001b6 }, KeysymCaseMapping { keysym: 0x03aa, other: 0x000003ba }, KeysymCaseMapping { keysym: 0x00d0, other: 0x000000f0 }, KeysymCaseMapping { keysym: 0x01ab, other: 0x000001bb }, KeysymCaseMapping { keysym: 0x03d1, other: 0x000003f1 }, KeysymCaseMapping { keysym: 0x06ea, other: 0x000006ca }, KeysymCaseMapping { keysym: 0x06be, other: 0x000006ae }, KeysymCaseMapping { keysym: 0x01c8, other: 0x000001e8 }, KeysymCaseMapping { keysym: 0x07ce, other: 0x000007ee }, KeysymCaseMapping { keysym: 0x07a9, other: 0x000007b9 }, KeysymCaseMapping { keysym: 0x06f7, other: 0x000006d7 }, KeysymCaseMapping { keysym: 0x03c7, other: 0x000003e7 }, KeysymCaseMapping { keysym: 0x03a6, other: 0x000003b6 }, KeysymCaseMapping { keysym: 0x02c5, other: 0x000002e5 }, KeysymCaseMapping { keysym: 0x01d8, other: 0x000001f8 }, KeysymCaseMapping { keysym: 0x07a1, other: 0x000007b1 }, KeysymCaseMapping { keysym: 0x01a1, other: 0x000001b1 }, KeysymCaseMapping { keysym: 0x00c1, other: 0x000000e1 }, KeysymCaseMapping { keysym: 0x07d8, other: 0x000007f8 }, KeysymCaseMapping { keysym: 0x06b2, other: 0x000006a2 }, KeysymCaseMapping { keysym: 0x00cb, other: 0x000000eb }, KeysymCaseMapping { keysym: 0x00db, other: 0x000000fb }, KeysymCaseMapping { keysym: 0x00d6, other: 0x000000f6 }, KeysymCaseMapping { keysym: 0x07c9, other: 0x000007e9 }, KeysymCaseMapping { keysym: 0x02c6, other: 0x000002e6 }, KeysymCaseMapping { keysym: 0x07c4, other: 0x000007e4 }, KeysymCaseMapping { keysym: 0x06b4, other: 0x000006a4 }, KeysymCaseMapping { keysym: 0x07a7, other: 0x000007b7 }, KeysymCaseMapping { keysym: 0x02a9, other: 0x00000069 }, KeysymCaseMapping { keysym: 0x06f2, other: 0x000006d2 }, KeysymCaseMapping { keysym: 0x03dd, other: 0x000003fd }, KeysymCaseMapping { keysym: 0x00ce, other: 0x000000ee }, KeysymCaseMapping { keysym: 0x03ab, other: 0x000003bb }, KeysymCaseMapping { keysym: 0x06b9, other: 0x000006a9 }, KeysymCaseMapping { keysym: 0x06e2, other: 0x000006c2 }, KeysymCaseMapping { keysym: 0x01ac, other: 0x000001bc }, KeysymCaseMapping { keysym: 0x06fc, other: 0x000006dc }, KeysymCaseMapping { keysym: 0x07a2, other: 0x000007b2 }, KeysymCaseMapping { keysym: 0x02d5, other: 0x000002f5 }, KeysymCaseMapping { keysym: 0x06bf, other: 0x000006af }, KeysymCaseMapping { keysym: 0x00d1, other: 0x000000f1 }, KeysymCaseMapping { keysym: 0x00cc, other: 0x000000ec }, KeysymCaseMapping { keysym: 0x06fd, other: 0x000006dd }, KeysymCaseMapping { keysym: 0x03d3, other: 0x000003f3 }, KeysymCaseMapping { keysym: 0x07d4, other: 0x000007f4 }, KeysymCaseMapping { keysym: 0x01c3, other: 0x000001e3 }, KeysymCaseMapping { keysym: 0x03de, other: 0x000003fe }, KeysymCaseMapping { keysym: 0x01d9, other: 0x000001f9 }, KeysymCaseMapping { keysym: 0x06e7, other: 0x000006c7 }, KeysymCaseMapping { keysym: 0x06e3, other: 0x000006c3 }, KeysymCaseMapping { keysym: 0x00dc, other: 0x000000fc }, KeysymCaseMapping { keysym: 0x07c5, other: 0x000007e5 }, KeysymCaseMapping { keysym: 0x06ed, other: 0x000006cd }, KeysymCaseMapping { keysym: 0x06ba, other: 0x000006aa }, KeysymCaseMapping { keysym: 0x07cf, other: 0x000007ef }, KeysymCaseMapping { keysym: 0x03cf, other: 0x000003ef }, KeysymCaseMapping { keysym: 0x06f3, other: 0x000006d3 }],
    _phantom: core::marker::PhantomData,
};

pub(super) static KEYSYM_TO_CHAR: &[KeysymChar] = &[
    KeysymChar {
        keysym: 0x0020,
        char: ' ',
    },
    KeysymChar {
        keysym: 0x0021,
        char: '!',
    },
    KeysymChar {
        keysym: 0x0022,
        char: '"',
    },
    KeysymChar {
        keysym: 0x0023,
        char: '#',
    },
    KeysymChar {
        keysym: 0x0024,
        char: '$',
    },
    KeysymChar {
        keysym: 0x0025,
        char: '%',
    },
    KeysymChar {
        keysym: 0x0026,
        char: '&',
    },
    KeysymChar {
        keysym: 0x0027,
        char: '\'',
    },
    KeysymChar {
        keysym: 0x0028,
        char: '(',
    },
    KeysymChar {
        keysym: 0x0029,
        char: ')',
    },
    KeysymChar {
        keysym: 0x002a,
        char: '*',
    },
    KeysymChar {
        keysym: 0x002b,
        char: '+',
    },
    KeysymChar {
        keysym: 0x002c,
        char: ',',
    },
    KeysymChar {
        keysym: 0x002d,
        char: '-',
    },
    KeysymChar {
        keysym: 0x002e,
        char: '.',
    },
    KeysymChar {
        keysym: 0x002f,
        char: '/',
    },
    KeysymChar {
        keysym: 0x0030,
        char: '0',
    },
    KeysymChar {
        keysym: 0x0031,
        char: '1',
    },
    KeysymChar {
        keysym: 0x0032,
        char: '2',
    },
    KeysymChar {
        keysym: 0x0033,
        char: '3',
    },
    KeysymChar {
        keysym: 0x0034,
        char: '4',
    },
    KeysymChar {
        keysym: 0x0035,
        char: '5',
    },
    KeysymChar {
        keysym: 0x0036,
        char: '6',
    },
    KeysymChar {
        keysym: 0x0037,
        char: '7',
    },
    KeysymChar {
        keysym: 0x0038,
        char: '8',
    },
    KeysymChar {
        keysym: 0x0039,
        char: '9',
    },
    KeysymChar {
        keysym: 0x003a,
        char: ':',
    },
    KeysymChar {
        keysym: 0x003b,
        char: ';',
    },
    KeysymChar {
        keysym: 0x003c,
        char: '<',
    },
    KeysymChar {
        keysym: 0x003d,
        char: '=',
    },
    KeysymChar {
        keysym: 0x003e,
        char: '>',
    },
    KeysymChar {
        keysym: 0x003f,
        char: '?',
    },
    KeysymChar {
        keysym: 0x0040,
        char: '@',
    },
    KeysymChar {
        keysym: 0x0041,
        char: 'A',
    },
    KeysymChar {
        keysym: 0x0042,
        char: 'B',
    },
    KeysymChar {
        keysym: 0x0043,
        char: 'C',
    },
    KeysymChar {
        keysym: 0x0044,
        char: 'D',
    },
    KeysymChar {
        keysym: 0x0045,
        char: 'E',
    },
    KeysymChar {
        keysym: 0x0046,
        char: 'F',
    },
    KeysymChar {
        keysym: 0x0047,
        char: 'G',
    },
    KeysymChar {
        keysym: 0x0048,
        char: 'H',
    },
    KeysymChar {
        keysym: 0x0049,
        char: 'I',
    },
    KeysymChar {
        keysym: 0x004a,
        char: 'J',
    },
    KeysymChar {
        keysym: 0x004b,
        char: 'K',
    },
    KeysymChar {
        keysym: 0x004c,
        char: 'L',
    },
    KeysymChar {
        keysym: 0x004d,
        char: 'M',
    },
    KeysymChar {
        keysym: 0x004e,
        char: 'N',
    },
    KeysymChar {
        keysym: 0x004f,
        char: 'O',
    },
    KeysymChar {
        keysym: 0x0050,
        char: 'P',
    },
    KeysymChar {
        keysym: 0x0051,
        char: 'Q',
    },
    KeysymChar {
        keysym: 0x0052,
        char: 'R',
    },
    KeysymChar {
        keysym: 0x0053,
        char: 'S',
    },
    KeysymChar {
        keysym: 0x0054,
        char: 'T',
    },
    KeysymChar {
        keysym: 0x0055,
        char: 'U',
    },
    KeysymChar {
        keysym: 0x0056,
        char: 'V',
    },
    KeysymChar {
        keysym: 0x0057,
        char: 'W',
    },
    KeysymChar {
        keysym: 0x0058,
        char: 'X',
    },
    KeysymChar {
        keysym: 0x0059,
        char: 'Y',
    },
    KeysymChar {
        keysym: 0x005a,
        char: 'Z',
    },
    KeysymChar {
        keysym: 0x005b,
        char: '[',
    },
    KeysymChar {
        keysym: 0x005c,
        char: '\\',
    },
    KeysymChar {
        keysym: 0x005d,
        char: ']',
    },
    KeysymChar {
        keysym: 0x005e,
        char: '^',
    },
    KeysymChar {
        keysym: 0x005f,
        char: '_',
    },
    KeysymChar {
        keysym: 0x0060,
        char: '`',
    },
    KeysymChar {
        keysym: 0x0061,
        char: 'a',
    },
    KeysymChar {
        keysym: 0x0062,
        char: 'b',
    },
    KeysymChar {
        keysym: 0x0063,
        char: 'c',
    },
    KeysymChar {
        keysym: 0x0064,
        char: 'd',
    },
    KeysymChar {
        keysym: 0x0065,
        char: 'e',
    },
    KeysymChar {
        keysym: 0x0066,
        char: 'f',
    },
    KeysymChar {
        keysym: 0x0067,
        char: 'g',
    },
    KeysymChar {
        keysym: 0x0068,
        char: 'h',
    },
    KeysymChar {
        keysym: 0x0069,
        char: 'i',
    },
    KeysymChar {
        keysym: 0x006a,
        char: 'j',
    },
    KeysymChar {
        keysym: 0x006b,
        char: 'k',
    },
    KeysymChar {
        keysym: 0x006c,
        char: 'l',
    },
    KeysymChar {
        keysym: 0x006d,
        char: 'm',
    },
    KeysymChar {
        keysym: 0x006e,
        char: 'n',
    },
    KeysymChar {
        keysym: 0x006f,
        char: 'o',
    },
    KeysymChar {
        keysym: 0x0070,
        char: 'p',
    },
    KeysymChar {
        keysym: 0x0071,
        char: 'q',
    },
    KeysymChar {
        keysym: 0x0072,
        char: 'r',
    },
    KeysymChar {
        keysym: 0x0073,
        char: 's',
    },
    KeysymChar {
        keysym: 0x0074,
        char: 't',
    },
    KeysymChar {
        keysym: 0x0075,
        char: 'u',
    },
    KeysymChar {
        keysym: 0x0076,
        char: 'v',
    },
    KeysymChar {
        keysym: 0x0077,
        char: 'w',
    },
    KeysymChar {
        keysym: 0x0078,
        char: 'x',
    },
    KeysymChar {
        keysym: 0x0079,
        char: 'y',
    },
    KeysymChar {
        keysym: 0x007a,
        char: 'z',
    },
    KeysymChar {
        keysym: 0x007b,
        char: '{',
    },
    KeysymChar {
        keysym: 0x007c,
        char: '|',
    },
    KeysymChar {
        keysym: 0x007d,
        char: '}',
    },
    KeysymChar {
        keysym: 0x007e,
        char: '~',
    },
    KeysymChar {
        keysym: 0x00a0,
        char: '\u{a0}',
    },
    KeysymChar {
        keysym: 0x00a1,
        char: '¡',
    },
    KeysymChar {
        keysym: 0x00a2,
        char: '¢',
    },
    KeysymChar {
        keysym: 0x00a3,
        char: '£',
    },
    KeysymChar {
        keysym: 0x00a4,
        char: '¤',
    },
    KeysymChar {
        keysym: 0x00a5,
        char: '¥',
    },
    KeysymChar {
        keysym: 0x00a6,
        char: '¦',
    },
    KeysymChar {
        keysym: 0x00a7,
        char: '§',
    },
    KeysymChar {
        keysym: 0x00a8,
        char: '¨',
    },
    KeysymChar {
        keysym: 0x00a9,
        char: '©',
    },
    KeysymChar {
        keysym: 0x00aa,
        char: 'ª',
    },
    KeysymChar {
        keysym: 0x00ab,
        char: '«',
    },
    KeysymChar {
        keysym: 0x00ac,
        char: '¬',
    },
    KeysymChar {
        keysym: 0x00ad,
        char: '\u{ad}',
    },
    KeysymChar {
        keysym: 0x00ae,
        char: '®',
    },
    KeysymChar {
        keysym: 0x00af,
        char: '¯',
    },
    KeysymChar {
        keysym: 0x00b0,
        char: '°',
    },
    KeysymChar {
        keysym: 0x00b1,
        char: '±',
    },
    KeysymChar {
        keysym: 0x00b2,
        char: '²',
    },
    KeysymChar {
        keysym: 0x00b3,
        char: '³',
    },
    KeysymChar {
        keysym: 0x00b4,
        char: '´',
    },
    KeysymChar {
        keysym: 0x00b5,
        char: 'µ',
    },
    KeysymChar {
        keysym: 0x00b6,
        char: '¶',
    },
    KeysymChar {
        keysym: 0x00b7,
        char: '·',
    },
    KeysymChar {
        keysym: 0x00b8,
        char: '¸',
    },
    KeysymChar {
        keysym: 0x00b9,
        char: '¹',
    },
    KeysymChar {
        keysym: 0x00ba,
        char: 'º',
    },
    KeysymChar {
        keysym: 0x00bb,
        char: '»',
    },
    KeysymChar {
        keysym: 0x00bc,
        char: '¼',
    },
    KeysymChar {
        keysym: 0x00bd,
        char: '½',
    },
    KeysymChar {
        keysym: 0x00be,
        char: '¾',
    },
    KeysymChar {
        keysym: 0x00bf,
        char: '¿',
    },
    KeysymChar {
        keysym: 0x00c0,
        char: 'À',
    },
    KeysymChar {
        keysym: 0x00c1,
        char: 'Á',
    },
    KeysymChar {
        keysym: 0x00c2,
        char: 'Â',
    },
    KeysymChar {
        keysym: 0x00c3,
        char: 'Ã',
    },
    KeysymChar {
        keysym: 0x00c4,
        char: 'Ä',
    },
    KeysymChar {
        keysym: 0x00c5,
        char: 'Å',
    },
    KeysymChar {
        keysym: 0x00c6,
        char: 'Æ',
    },
    KeysymChar {
        keysym: 0x00c7,
        char: 'Ç',
    },
    KeysymChar {
        keysym: 0x00c8,
        char: 'È',
    },
    KeysymChar {
        keysym: 0x00c9,
        char: 'É',
    },
    KeysymChar {
        keysym: 0x00ca,
        char: 'Ê',
    },
    KeysymChar {
        keysym: 0x00cb,
        char: 'Ë',
    },
    KeysymChar {
        keysym: 0x00cc,
        char: 'Ì',
    },
    KeysymChar {
        keysym: 0x00cd,
        char: 'Í',
    },
    KeysymChar {
        keysym: 0x00ce,
        char: 'Î',
    },
    KeysymChar {
        keysym: 0x00cf,
        char: 'Ï',
    },
    KeysymChar {
        keysym: 0x00d0,
        char: 'Ð',
    },
    KeysymChar {
        keysym: 0x00d1,
        char: 'Ñ',
    },
    KeysymChar {
        keysym: 0x00d2,
        char: 'Ò',
    },
    KeysymChar {
        keysym: 0x00d3,
        char: 'Ó',
    },
    KeysymChar {
        keysym: 0x00d4,
        char: 'Ô',
    },
    KeysymChar {
        keysym: 0x00d5,
        char: 'Õ',
    },
    KeysymChar {
        keysym: 0x00d6,
        char: 'Ö',
    },
    KeysymChar {
        keysym: 0x00d7,
        char: '×',
    },
    KeysymChar {
        keysym: 0x00d8,
        char: 'Ø',
    },
    KeysymChar {
        keysym: 0x00d9,
        char: 'Ù',
    },
    KeysymChar {
        keysym: 0x00da,
        char: 'Ú',
    },
    KeysymChar {
        keysym: 0x00db,
        char: 'Û',
    },
    KeysymChar {
        keysym: 0x00dc,
        char: 'Ü',
    },
    KeysymChar {
        keysym: 0x00dd,
        char: 'Ý',
    },
    KeysymChar {
        keysym: 0x00de,
        char: 'Þ',
    },
    KeysymChar {
        keysym: 0x00df,
        char: 'ß',
    },
    KeysymChar {
        keysym: 0x00e0,
        char: 'à',
    },
    KeysymChar {
        keysym: 0x00e1,
        char: 'á',
    },
    KeysymChar {
        keysym: 0x00e2,
        char: 'â',
    },
    KeysymChar {
        keysym: 0x00e3,
        char: 'ã',
    },
    KeysymChar {
        keysym: 0x00e4,
        char: 'ä',
    },
    KeysymChar {
        keysym: 0x00e5,
        char: 'å',
    },
    KeysymChar {
        keysym: 0x00e6,
        char: 'æ',
    },
    KeysymChar {
        keysym: 0x00e7,
        char: 'ç',
    },
    KeysymChar {
        keysym: 0x00e8,
        char: 'è',
    },
    KeysymChar {
        keysym: 0x00e9,
        char: 'é',
    },
    KeysymChar {
        keysym: 0x00ea,
        char: 'ê',
    },
    KeysymChar {
        keysym: 0x00eb,
        char: 'ë',
    },
    KeysymChar {
        keysym: 0x00ec,
        char: 'ì',
    },
    KeysymChar {
        keysym: 0x00ed,
        char: 'í',
    },
    KeysymChar {
        keysym: 0x00ee,
        char: 'î',
    },
    KeysymChar {
        keysym: 0x00ef,
        char: 'ï',
    },
    KeysymChar {
        keysym: 0x00f0,
        char: 'ð',
    },
    KeysymChar {
        keysym: 0x00f1,
        char: 'ñ',
    },
    KeysymChar {
        keysym: 0x00f2,
        char: 'ò',
    },
    KeysymChar {
        keysym: 0x00f3,
        char: 'ó',
    },
    KeysymChar {
        keysym: 0x00f4,
        char: 'ô',
    },
    KeysymChar {
        keysym: 0x00f5,
        char: 'õ',
    },
    KeysymChar {
        keysym: 0x00f6,
        char: 'ö',
    },
    KeysymChar {
        keysym: 0x00f7,
        char: '÷',
    },
    KeysymChar {
        keysym: 0x00f8,
        char: 'ø',
    },
    KeysymChar {
        keysym: 0x00f9,
        char: 'ù',
    },
    KeysymChar {
        keysym: 0x00fa,
        char: 'ú',
    },
    KeysymChar {
        keysym: 0x00fb,
        char: 'û',
    },
    KeysymChar {
        keysym: 0x00fc,
        char: 'ü',
    },
    KeysymChar {
        keysym: 0x00fd,
        char: 'ý',
    },
    KeysymChar {
        keysym: 0x00fe,
        char: 'þ',
    },
    KeysymChar {
        keysym: 0x00ff,
        char: 'ÿ',
    },
    KeysymChar {
        keysym: 0x01a1,
        char: 'Ą',
    },
    KeysymChar {
        keysym: 0x01a2,
        char: '˘',
    },
    KeysymChar {
        keysym: 0x01a3,
        char: 'Ł',
    },
    KeysymChar {
        keysym: 0x01a5,
        char: 'Ľ',
    },
    KeysymChar {
        keysym: 0x01a6,
        char: 'Ś',
    },
    KeysymChar {
        keysym: 0x01a9,
        char: 'Š',
    },
    KeysymChar {
        keysym: 0x01aa,
        char: 'Ş',
    },
    KeysymChar {
        keysym: 0x01ab,
        char: 'Ť',
    },
    KeysymChar {
        keysym: 0x01ac,
        char: 'Ź',
    },
    KeysymChar {
        keysym: 0x01ae,
        char: 'Ž',
    },
    KeysymChar {
        keysym: 0x01af,
        char: 'Ż',
    },
    KeysymChar {
        keysym: 0x01b1,
        char: 'ą',
    },
    KeysymChar {
        keysym: 0x01b2,
        char: '˛',
    },
    KeysymChar {
        keysym: 0x01b3,
        char: 'ł',
    },
    KeysymChar {
        keysym: 0x01b5,
        char: 'ľ',
    },
    KeysymChar {
        keysym: 0x01b6,
        char: 'ś',
    },
    KeysymChar {
        keysym: 0x01b7,
        char: 'ˇ',
    },
    KeysymChar {
        keysym: 0x01b9,
        char: 'š',
    },
    KeysymChar {
        keysym: 0x01ba,
        char: 'ş',
    },
    KeysymChar {
        keysym: 0x01bb,
        char: 'ť',
    },
    KeysymChar {
        keysym: 0x01bc,
        char: 'ź',
    },
    KeysymChar {
        keysym: 0x01bd,
        char: '˝',
    },
    KeysymChar {
        keysym: 0x01be,
        char: 'ž',
    },
    KeysymChar {
        keysym: 0x01bf,
        char: 'ż',
    },
    KeysymChar {
        keysym: 0x01c0,
        char: 'Ŕ',
    },
    KeysymChar {
        keysym: 0x01c3,
        char: 'Ă',
    },
    KeysymChar {
        keysym: 0x01c5,
        char: 'Ĺ',
    },
    KeysymChar {
        keysym: 0x01c6,
        char: 'Ć',
    },
    KeysymChar {
        keysym: 0x01c8,
        char: 'Č',
    },
    KeysymChar {
        keysym: 0x01ca,
        char: 'Ę',
    },
    KeysymChar {
        keysym: 0x01cc,
        char: 'Ě',
    },
    KeysymChar {
        keysym: 0x01cf,
        char: 'Ď',
    },
    KeysymChar {
        keysym: 0x01d0,
        char: 'Đ',
    },
    KeysymChar {
        keysym: 0x01d1,
        char: 'Ń',
    },
    KeysymChar {
        keysym: 0x01d2,
        char: 'Ň',
    },
    KeysymChar {
        keysym: 0x01d5,
        char: 'Ő',
    },
    KeysymChar {
        keysym: 0x01d8,
        char: 'Ř',
    },
    KeysymChar {
        keysym: 0x01d9,
        char: 'Ů',
    },
    KeysymChar {
        keysym: 0x01db,
        char: 'Ű',
    },
    KeysymChar {
        keysym: 0x01de,
        char: 'Ţ',
    },
    KeysymChar {
        keysym: 0x01e0,
        char: 'ŕ',
    },
    KeysymChar {
        keysym: 0x01e3,
        char: 'ă',
    },
    KeysymChar {
        keysym: 0x01e5,
        char: 'ĺ',
    },
    KeysymChar {
        keysym: 0x01e6,
        char: 'ć',
    },
    KeysymChar {
        keysym: 0x01e8,
        char: 'č',
    },
    KeysymChar {
        keysym: 0x01ea,
        char: 'ę',
    },
    KeysymChar {
        keysym: 0x01ec,
        char: 'ě',
    },
    KeysymChar {
        keysym: 0x01ef,
        char: 'ď',
    },
    KeysymChar {
        keysym: 0x01f0,
        char: 'đ',
    },
    KeysymChar {
        keysym: 0x01f1,
        char: 'ń',
    },
    KeysymChar {
        keysym: 0x01f2,
        char: 'ň',
    },
    KeysymChar {
        keysym: 0x01f5,
        char: 'ő',
    },
    KeysymChar {
        keysym: 0x01f8,
        char: 'ř',
    },
    KeysymChar {
        keysym: 0x01f9,
        char: 'ů',
    },
    KeysymChar {
        keysym: 0x01fb,
        char: 'ű',
    },
    KeysymChar {
        keysym: 0x01fe,
        char: 'ţ',
    },
    KeysymChar {
        keysym: 0x01ff,
        char: '˙',
    },
    KeysymChar {
        keysym: 0x02a1,
        char: 'Ħ',
    },
    KeysymChar {
        keysym: 0x02a6,
        char: 'Ĥ',
    },
    KeysymChar {
        keysym: 0x02a9,
        char: 'İ',
    },
    KeysymChar {
        keysym: 0x02ab,
        char: 'Ğ',
    },
    KeysymChar {
        keysym: 0x02ac,
        char: 'Ĵ',
    },
    KeysymChar {
        keysym: 0x02b1,
        char: 'ħ',
    },
    KeysymChar {
        keysym: 0x02b6,
        char: 'ĥ',
    },
    KeysymChar {
        keysym: 0x02b9,
        char: 'ı',
    },
    KeysymChar {
        keysym: 0x02bb,
        char: 'ğ',
    },
    KeysymChar {
        keysym: 0x02bc,
        char: 'ĵ',
    },
    KeysymChar {
        keysym: 0x02c5,
        char: 'Ċ',
    },
    KeysymChar {
        keysym: 0x02c6,
        char: 'Ĉ',
    },
    KeysymChar {
        keysym: 0x02d5,
        char: 'Ġ',
    },
    KeysymChar {
        keysym: 0x02d8,
        char: 'Ĝ',
    },
    KeysymChar {
        keysym: 0x02dd,
        char: 'Ŭ',
    },
    KeysymChar {
        keysym: 0x02de,
        char: 'Ŝ',
    },
    KeysymChar {
        keysym: 0x02e5,
        char: 'ċ',
    },
    KeysymChar {
        keysym: 0x02e6,
        char: 'ĉ',
    },
    KeysymChar {
        keysym: 0x02f5,
        char: 'ġ',
    },
    KeysymChar {
        keysym: 0x02f8,
        char: 'ĝ',
    },
    KeysymChar {
        keysym: 0x02fd,
        char: 'ŭ',
    },
    KeysymChar {
        keysym: 0x02fe,
        char: 'ŝ',
    },
    KeysymChar {
        keysym: 0x03a2,
        char: 'ĸ',
    },
    KeysymChar {
        keysym: 0x03a3,
        char: 'Ŗ',
    },
    KeysymChar {
        keysym: 0x03a5,
        char: 'Ĩ',
    },
    KeysymChar {
        keysym: 0x03a6,
        char: 'Ļ',
    },
    KeysymChar {
        keysym: 0x03aa,
        char: 'Ē',
    },
    KeysymChar {
        keysym: 0x03ab,
        char: 'Ģ',
    },
    KeysymChar {
        keysym: 0x03ac,
        char: 'Ŧ',
    },
    KeysymChar {
        keysym: 0x03b3,
        char: 'ŗ',
    },
    KeysymChar {
        keysym: 0x03b5,
        char: 'ĩ',
    },
    KeysymChar {
        keysym: 0x03b6,
        char: 'ļ',
    },
    KeysymChar {
        keysym: 0x03ba,
        char: 'ē',
    },
    KeysymChar {
        keysym: 0x03bb,
        char: 'ģ',
    },
    KeysymChar {
        keysym: 0x03bc,
        char: 'ŧ',
    },
    KeysymChar {
        keysym: 0x03bd,
        char: 'Ŋ',
    },
    KeysymChar {
        keysym: 0x03bf,
        char: 'ŋ',
    },
    KeysymChar {
        keysym: 0x03c0,
        char: 'Ā',
    },
    KeysymChar {
        keysym: 0x03c7,
        char: 'Į',
    },
    KeysymChar {
        keysym: 0x03cc,
        char: 'Ė',
    },
    KeysymChar {
        keysym: 0x03cf,
        char: 'Ī',
    },
    KeysymChar {
        keysym: 0x03d1,
        char: 'Ņ',
    },
    KeysymChar {
        keysym: 0x03d2,
        char: 'Ō',
    },
    KeysymChar {
        keysym: 0x03d3,
        char: 'Ķ',
    },
    KeysymChar {
        keysym: 0x03d9,
        char: 'Ų',
    },
    KeysymChar {
        keysym: 0x03dd,
        char: 'Ũ',
    },
    KeysymChar {
        keysym: 0x03de,
        char: 'Ū',
    },
    KeysymChar {
        keysym: 0x03e0,
        char: 'ā',
    },
    KeysymChar {
        keysym: 0x03e7,
        char: 'į',
    },
    KeysymChar {
        keysym: 0x03ec,
        char: 'ė',
    },
    KeysymChar {
        keysym: 0x03ef,
        char: 'ī',
    },
    KeysymChar {
        keysym: 0x03f1,
        char: 'ņ',
    },
    KeysymChar {
        keysym: 0x03f2,
        char: 'ō',
    },
    KeysymChar {
        keysym: 0x03f3,
        char: 'ķ',
    },
    KeysymChar {
        keysym: 0x03f9,
        char: 'ų',
    },
    KeysymChar {
        keysym: 0x03fd,
        char: 'ũ',
    },
    KeysymChar {
        keysym: 0x03fe,
        char: 'ū',
    },
    KeysymChar {
        keysym: 0x047e,
        char: '‾',
    },
    KeysymChar {
        keysym: 0x04a1,
        char: '。',
    },
    KeysymChar {
        keysym: 0x04a2,
        char: '「',
    },
    KeysymChar {
        keysym: 0x04a3,
        char: '」',
    },
    KeysymChar {
        keysym: 0x04a4,
        char: '、',
    },
    KeysymChar {
        keysym: 0x04a5,
        char: '・',
    },
    KeysymChar {
        keysym: 0x04a6,
        char: 'ヲ',
    },
    KeysymChar {
        keysym: 0x04a7,
        char: 'ァ',
    },
    KeysymChar {
        keysym: 0x04a8,
        char: 'ィ',
    },
    KeysymChar {
        keysym: 0x04a9,
        char: 'ゥ',
    },
    KeysymChar {
        keysym: 0x04aa,
        char: 'ェ',
    },
    KeysymChar {
        keysym: 0x04ab,
        char: 'ォ',
    },
    KeysymChar {
        keysym: 0x04ac,
        char: 'ャ',
    },
    KeysymChar {
        keysym: 0x04ad,
        char: 'ュ',
    },
    KeysymChar {
        keysym: 0x04ae,
        char: 'ョ',
    },
    KeysymChar {
        keysym: 0x04af,
        char: 'ッ',
    },
    KeysymChar {
        keysym: 0x04b0,
        char: 'ー',
    },
    KeysymChar {
        keysym: 0x04b1,
        char: 'ア',
    },
    KeysymChar {
        keysym: 0x04b2,
        char: 'イ',
    },
    KeysymChar {
        keysym: 0x04b3,
        char: 'ウ',
    },
    KeysymChar {
        keysym: 0x04b4,
        char: 'エ',
    },
    KeysymChar {
        keysym: 0x04b5,
        char: 'オ',
    },
    KeysymChar {
        keysym: 0x04b6,
        char: 'カ',
    },
    KeysymChar {
        keysym: 0x04b7,
        char: 'キ',
    },
    KeysymChar {
        keysym: 0x04b8,
        char: 'ク',
    },
    KeysymChar {
        keysym: 0x04b9,
        char: 'ケ',
    },
    KeysymChar {
        keysym: 0x04ba,
        char: 'コ',
    },
    KeysymChar {
        keysym: 0x04bb,
        char: 'サ',
    },
    KeysymChar {
        keysym: 0x04bc,
        char: 'シ',
    },
    KeysymChar {
        keysym: 0x04bd,
        char: 'ス',
    },
    KeysymChar {
        keysym: 0x04be,
        char: 'セ',
    },
    KeysymChar {
        keysym: 0x04bf,
        char: 'ソ',
    },
    KeysymChar {
        keysym: 0x04c0,
        char: 'タ',
    },
    KeysymChar {
        keysym: 0x04c1,
        char: 'チ',
    },
    KeysymChar {
        keysym: 0x04c2,
        char: 'ツ',
    },
    KeysymChar {
        keysym: 0x04c3,
        char: 'テ',
    },
    KeysymChar {
        keysym: 0x04c4,
        char: 'ト',
    },
    KeysymChar {
        keysym: 0x04c5,
        char: 'ナ',
    },
    KeysymChar {
        keysym: 0x04c6,
        char: 'ニ',
    },
    KeysymChar {
        keysym: 0x04c7,
        char: 'ヌ',
    },
    KeysymChar {
        keysym: 0x04c8,
        char: 'ネ',
    },
    KeysymChar {
        keysym: 0x04c9,
        char: 'ノ',
    },
    KeysymChar {
        keysym: 0x04ca,
        char: 'ハ',
    },
    KeysymChar {
        keysym: 0x04cb,
        char: 'ヒ',
    },
    KeysymChar {
        keysym: 0x04cc,
        char: 'フ',
    },
    KeysymChar {
        keysym: 0x04cd,
        char: 'ヘ',
    },
    KeysymChar {
        keysym: 0x04ce,
        char: 'ホ',
    },
    KeysymChar {
        keysym: 0x04cf,
        char: 'マ',
    },
    KeysymChar {
        keysym: 0x04d0,
        char: 'ミ',
    },
    KeysymChar {
        keysym: 0x04d1,
        char: 'ム',
    },
    KeysymChar {
        keysym: 0x04d2,
        char: 'メ',
    },
    KeysymChar {
        keysym: 0x04d3,
        char: 'モ',
    },
    KeysymChar {
        keysym: 0x04d4,
        char: 'ヤ',
    },
    KeysymChar {
        keysym: 0x04d5,
        char: 'ユ',
    },
    KeysymChar {
        keysym: 0x04d6,
        char: 'ヨ',
    },
    KeysymChar {
        keysym: 0x04d7,
        char: 'ラ',
    },
    KeysymChar {
        keysym: 0x04d8,
        char: 'リ',
    },
    KeysymChar {
        keysym: 0x04d9,
        char: 'ル',
    },
    KeysymChar {
        keysym: 0x04da,
        char: 'レ',
    },
    KeysymChar {
        keysym: 0x04db,
        char: 'ロ',
    },
    KeysymChar {
        keysym: 0x04dc,
        char: 'ワ',
    },
    KeysymChar {
        keysym: 0x04dd,
        char: 'ン',
    },
    KeysymChar {
        keysym: 0x04de,
        char: '゛',
    },
    KeysymChar {
        keysym: 0x04df,
        char: '゜',
    },
    KeysymChar {
        keysym: 0x05ac,
        char: '،',
    },
    KeysymChar {
        keysym: 0x05bb,
        char: '؛',
    },
    KeysymChar {
        keysym: 0x05bf,
        char: '؟',
    },
    KeysymChar {
        keysym: 0x05c1,
        char: 'ء',
    },
    KeysymChar {
        keysym: 0x05c2,
        char: 'آ',
    },
    KeysymChar {
        keysym: 0x05c3,
        char: 'أ',
    },
    KeysymChar {
        keysym: 0x05c4,
        char: 'ؤ',
    },
    KeysymChar {
        keysym: 0x05c5,
        char: 'إ',
    },
    KeysymChar {
        keysym: 0x05c6,
        char: 'ئ',
    },
    KeysymChar {
        keysym: 0x05c7,
        char: 'ا',
    },
    KeysymChar {
        keysym: 0x05c8,
        char: 'ب',
    },
    KeysymChar {
        keysym: 0x05c9,
        char: 'ة',
    },
    KeysymChar {
        keysym: 0x05ca,
        char: 'ت',
    },
    KeysymChar {
        keysym: 0x05cb,
        char: 'ث',
    },
    KeysymChar {
        keysym: 0x05cc,
        char: 'ج',
    },
    KeysymChar {
        keysym: 0x05cd,
        char: 'ح',
    },
    KeysymChar {
        keysym: 0x05ce,
        char: 'خ',
    },
    KeysymChar {
        keysym: 0x05cf,
        char: 'د',
    },
    KeysymChar {
        keysym: 0x05d0,
        char: 'ذ',
    },
    KeysymChar {
        keysym: 0x05d1,
        char: 'ر',
    },
    KeysymChar {
        keysym: 0x05d2,
        char: 'ز',
    },
    KeysymChar {
        keysym: 0x05d3,
        char: 'س',
    },
    KeysymChar {
        keysym: 0x05d4,
        char: 'ش',
    },
    KeysymChar {
        keysym: 0x05d5,
        char: 'ص',
    },
    KeysymChar {
        keysym: 0x05d6,
        char: 'ض',
    },
    KeysymChar {
        keysym: 0x05d7,
        char: 'ط',
    },
    KeysymChar {
        keysym: 0x05d8,
        char: 'ظ',
    },
    KeysymChar {
        keysym: 0x05d9,
        char: 'ع',
    },
    KeysymChar {
        keysym: 0x05da,
        char: 'غ',
    },
    KeysymChar {
        keysym: 0x05e0,
        char: 'ـ',
    },
    KeysymChar {
        keysym: 0x05e1,
        char: 'ف',
    },
    KeysymChar {
        keysym: 0x05e2,
        char: 'ق',
    },
    KeysymChar {
        keysym: 0x05e3,
        char: 'ك',
    },
    KeysymChar {
        keysym: 0x05e4,
        char: 'ل',
    },
    KeysymChar {
        keysym: 0x05e5,
        char: 'م',
    },
    KeysymChar {
        keysym: 0x05e6,
        char: 'ن',
    },
    KeysymChar {
        keysym: 0x05e7,
        char: 'ه',
    },
    KeysymChar {
        keysym: 0x05e8,
        char: 'و',
    },
    KeysymChar {
        keysym: 0x05e9,
        char: 'ى',
    },
    KeysymChar {
        keysym: 0x05ea,
        char: 'ي',
    },
    KeysymChar {
        keysym: 0x05eb,
        char: '\u{64b}',
    },
    KeysymChar {
        keysym: 0x05ec,
        char: '\u{64c}',
    },
    KeysymChar {
        keysym: 0x05ed,
        char: '\u{64d}',
    },
    KeysymChar {
        keysym: 0x05ee,
        char: '\u{64e}',
    },
    KeysymChar {
        keysym: 0x05ef,
        char: '\u{64f}',
    },
    KeysymChar {
        keysym: 0x05f0,
        char: '\u{650}',
    },
    KeysymChar {
        keysym: 0x05f1,
        char: '\u{651}',
    },
    KeysymChar {
        keysym: 0x05f2,
        char: '\u{652}',
    },
    KeysymChar {
        keysym: 0x06a1,
        char: 'ђ',
    },
    KeysymChar {
        keysym: 0x06a2,
        char: 'ѓ',
    },
    KeysymChar {
        keysym: 0x06a3,
        char: 'ё',
    },
    KeysymChar {
        keysym: 0x06a4,
        char: 'є',
    },
    KeysymChar {
        keysym: 0x06a5,
        char: 'ѕ',
    },
    KeysymChar {
        keysym: 0x06a6,
        char: 'і',
    },
    KeysymChar {
        keysym: 0x06a7,
        char: 'ї',
    },
    KeysymChar {
        keysym: 0x06a8,
        char: 'ј',
    },
    KeysymChar {
        keysym: 0x06a9,
        char: 'љ',
    },
    KeysymChar {
        keysym: 0x06aa,
        char: 'њ',
    },
    KeysymChar {
        keysym: 0x06ab,
        char: 'ћ',
    },
    KeysymChar {
        keysym: 0x06ac,
        char: 'ќ',
    },
    KeysymChar {
        keysym: 0x06ad,
        char: 'ґ',
    },
    KeysymChar {
        keysym: 0x06ae,
        char: 'ў',
    },
    KeysymChar {
        keysym: 0x06af,
        char: 'џ',
    },
    KeysymChar {
        keysym: 0x06b0,
        char: '№',
    },
    KeysymChar {
        keysym: 0x06b1,
        char: 'Ђ',
    },
    KeysymChar {
        keysym: 0x06b2,
        char: 'Ѓ',
    },
    KeysymChar {
        keysym: 0x06b3,
        char: 'Ё',
    },
    KeysymChar {
        keysym: 0x06b4,
        char: 'Є',
    },
    KeysymChar {
        keysym: 0x06b5,
        char: 'Ѕ',
    },
    KeysymChar {
        keysym: 0x06b6,
        char: 'І',
    },
    KeysymChar {
        keysym: 0x06b7,
        char: 'Ї',
    },
    KeysymChar {
        keysym: 0x06b8,
        char: 'Ј',
    },
    KeysymChar {
        keysym: 0x06b9,
        char: 'Љ',
    },
    KeysymChar {
        keysym: 0x06ba,
        char: 'Њ',
    },
    KeysymChar {
        keysym: 0x06bb,
        char: 'Ћ',
    },
    KeysymChar {
        keysym: 0x06bc,
        char: 'Ќ',
    },
    KeysymChar {
        keysym: 0x06bd,
        char: 'Ґ',
    },
    KeysymChar {
        keysym: 0x06be,
        char: 'Ў',
    },
    KeysymChar {
        keysym: 0x06bf,
        char: 'Џ',
    },
    KeysymChar {
        keysym: 0x06c0,
        char: 'ю',
    },
    KeysymChar {
        keysym: 0x06c1,
        char: 'а',
    },
    KeysymChar {
        keysym: 0x06c2,
        char: 'б',
    },
    KeysymChar {
        keysym: 0x06c3,
        char: 'ц',
    },
    KeysymChar {
        keysym: 0x06c4,
        char: 'д',
    },
    KeysymChar {
        keysym: 0x06c5,
        char: 'е',
    },
    KeysymChar {
        keysym: 0x06c6,
        char: 'ф',
    },
    KeysymChar {
        keysym: 0x06c7,
        char: 'г',
    },
    KeysymChar {
        keysym: 0x06c8,
        char: 'х',
    },
    KeysymChar {
        keysym: 0x06c9,
        char: 'и',
    },
    KeysymChar {
        keysym: 0x06ca,
        char: 'й',
    },
    KeysymChar {
        keysym: 0x06cb,
        char: 'к',
    },
    KeysymChar {
        keysym: 0x06cc,
        char: 'л',
    },
    KeysymChar {
        keysym: 0x06cd,
        char: 'м',
    },
    KeysymChar {
        keysym: 0x06ce,
        char: 'н',
    },
    KeysymChar {
        keysym: 0x06cf,
        char: 'о',
    },
    KeysymChar {
        keysym: 0x06d0,
        char: 'п',
    },
    KeysymChar {
        keysym: 0x06d1,
        char: 'я',
    },
    KeysymChar {
        keysym: 0x06d2,
        char: 'р',
    },
    KeysymChar {
        keysym: 0x06d3,
        char: 'с',
    },
    KeysymChar {
        keysym: 0x06d4,
        char: 'т',
    },
    KeysymChar {
        keysym: 0x06d5,
        char: 'у',
    },
    KeysymChar {
        keysym: 0x06d6,
        char: 'ж',
    },
    KeysymChar {
        keysym: 0x06d7,
        char: 'в',
    },
    KeysymChar {
        keysym: 0x06d8,
        char: 'ь',
    },
    KeysymChar {
        keysym: 0x06d9,
        char: 'ы',
    },
    KeysymChar {
        keysym: 0x06da,
        char: 'з',
    },
    KeysymChar {
        keysym: 0x06db,
        char: 'ш',
    },
    KeysymChar {
        keysym: 0x06dc,
        char: 'э',
    },
    KeysymChar {
        keysym: 0x06dd,
        char: 'щ',
    },
    KeysymChar {
        keysym: 0x06de,
        char: 'ч',
    },
    KeysymChar {
        keysym: 0x06df,
        char: 'ъ',
    },
    KeysymChar {
        keysym: 0x06e0,
        char: 'Ю',
    },
    KeysymChar {
        keysym: 0x06e1,
        char: 'А',
    },
    KeysymChar {
        keysym: 0x06e2,
        char: 'Б',
    },
    KeysymChar {
        keysym: 0x06e3,
        char: 'Ц',
    },
    KeysymChar {
        keysym: 0x06e4,
        char: 'Д',
    },
    KeysymChar {
        keysym: 0x06e5,
        char: 'Е',
    },
    KeysymChar {
        keysym: 0x06e6,
        char: 'Ф',
    },
    KeysymChar {
        keysym: 0x06e7,
        char: 'Г',
    },
    KeysymChar {
        keysym: 0x06e8,
        char: 'Х',
    },
    KeysymChar {
        keysym: 0x06e9,
        char: 'И',
    },
    KeysymChar {
        keysym: 0x06ea,
        char: 'Й',
    },
    KeysymChar {
        keysym: 0x06eb,
        char: 'К',
    },
    KeysymChar {
        keysym: 0x06ec,
        char: 'Л',
    },
    KeysymChar {
        keysym: 0x06ed,
        char: 'М',
    },
    KeysymChar {
        keysym: 0x06ee,
        char: 'Н',
    },
    KeysymChar {
        keysym: 0x06ef,
        char: 'О',
    },
    KeysymChar {
        keysym: 0x06f0,
        char: 'П',
    },
    KeysymChar {
        keysym: 0x06f1,
        char: 'Я',
    },
    KeysymChar {
        keysym: 0x06f2,
        char: 'Р',
    },
    KeysymChar {
        keysym: 0x06f3,
        char: 'С',
    },
    KeysymChar {
        keysym: 0x06f4,
        char: 'Т',
    },
    KeysymChar {
        keysym: 0x06f5,
        char: 'У',
    },
    KeysymChar {
        keysym: 0x06f6,
        char: 'Ж',
    },
    KeysymChar {
        keysym: 0x06f7,
        char: 'В',
    },
    KeysymChar {
        keysym: 0x06f8,
        char: 'Ь',
    },
    KeysymChar {
        keysym: 0x06f9,
        char: 'Ы',
    },
    KeysymChar {
        keysym: 0x06fa,
        char: 'З',
    },
    KeysymChar {
        keysym: 0x06fb,
        char: 'Ш',
    },
    KeysymChar {
        keysym: 0x06fc,
        char: 'Э',
    },
    KeysymChar {
        keysym: 0x06fd,
        char: 'Щ',
    },
    KeysymChar {
        keysym: 0x06fe,
        char: 'Ч',
    },
    KeysymChar {
        keysym: 0x06ff,
        char: 'Ъ',
    },
    KeysymChar {
        keysym: 0x07a1,
        char: 'Ά',
    },
    KeysymChar {
        keysym: 0x07a2,
        char: 'Έ',
    },
    KeysymChar {
        keysym: 0x07a3,
        char: 'Ή',
    },
    KeysymChar {
        keysym: 0x07a4,
        char: 'Ί',
    },
    KeysymChar {
        keysym: 0x07a5,
        char: 'Ϊ',
    },
    KeysymChar {
        keysym: 0x07a7,
        char: 'Ό',
    },
    KeysymChar {
        keysym: 0x07a8,
        char: 'Ύ',
    },
    KeysymChar {
        keysym: 0x07a9,
        char: 'Ϋ',
    },
    KeysymChar {
        keysym: 0x07ab,
        char: 'Ώ',
    },
    KeysymChar {
        keysym: 0x07ae,
        char: '΅',
    },
    KeysymChar {
        keysym: 0x07af,
        char: '―',
    },
    KeysymChar {
        keysym: 0x07b1,
        char: 'ά',
    },
    KeysymChar {
        keysym: 0x07b2,
        char: 'έ',
    },
    KeysymChar {
        keysym: 0x07b3,
        char: 'ή',
    },
    KeysymChar {
        keysym: 0x07b4,
        char: 'ί',
    },
    KeysymChar {
        keysym: 0x07b5,
        char: 'ϊ',
    },
    KeysymChar {
        keysym: 0x07b6,
        char: 'ΐ',
    },
    KeysymChar {
        keysym: 0x07b7,
        char: 'ό',
    },
    KeysymChar {
        keysym: 0x07b8,
        char: 'ύ',
    },
    KeysymChar {
        keysym: 0x07b9,
        char: 'ϋ',
    },
    KeysymChar {
        keysym: 0x07ba,
        char: 'ΰ',
    },
    KeysymChar {
        keysym: 0x07bb,
        char: 'ώ',
    },
    KeysymChar {
        keysym: 0x07c1,
        char: 'Α',
    },
    KeysymChar {
        keysym: 0x07c2,
        char: 'Β',
    },
    KeysymChar {
        keysym: 0x07c3,
        char: 'Γ',
    },
    KeysymChar {
        keysym: 0x07c4,
        char: 'Δ',
    },
    KeysymChar {
        keysym: 0x07c5,
        char: 'Ε',
    },
    KeysymChar {
        keysym: 0x07c6,
        char: 'Ζ',
    },
    KeysymChar {
        keysym: 0x07c7,
        char: 'Η',
    },
    KeysymChar {
        keysym: 0x07c8,
        char: 'Θ',
    },
    KeysymChar {
        keysym: 0x07c9,
        char: 'Ι',
    },
    KeysymChar {
        keysym: 0x07ca,
        char: 'Κ',
    },
    KeysymChar {
        keysym: 0x07cb,
        char: 'Λ',
    },
    KeysymChar {
        keysym: 0x07cc,
        char: 'Μ',
    },
    KeysymChar {
        keysym: 0x07cd,
        char: 'Ν',
    },
    KeysymChar {
        keysym: 0x07ce,
        char: 'Ξ',
    },
    KeysymChar {
        keysym: 0x07cf,
        char: 'Ο',
    },
    KeysymChar {
        keysym: 0x07d0,
        char: 'Π',
    },
    KeysymChar {
        keysym: 0x07d1,
        char: 'Ρ',
    },
    KeysymChar {
        keysym: 0x07d2,
        char: 'Σ',
    },
    KeysymChar {
        keysym: 0x07d4,
        char: 'Τ',
    },
    KeysymChar {
        keysym: 0x07d5,
        char: 'Υ',
    },
    KeysymChar {
        keysym: 0x07d6,
        char: 'Φ',
    },
    KeysymChar {
        keysym: 0x07d7,
        char: 'Χ',
    },
    KeysymChar {
        keysym: 0x07d8,
        char: 'Ψ',
    },
    KeysymChar {
        keysym: 0x07d9,
        char: 'Ω',
    },
    KeysymChar {
        keysym: 0x07e1,
        char: 'α',
    },
    KeysymChar {
        keysym: 0x07e2,
        char: 'β',
    },
    KeysymChar {
        keysym: 0x07e3,
        char: 'γ',
    },
    KeysymChar {
        keysym: 0x07e4,
        char: 'δ',
    },
    KeysymChar {
        keysym: 0x07e5,
        char: 'ε',
    },
    KeysymChar {
        keysym: 0x07e6,
        char: 'ζ',
    },
    KeysymChar {
        keysym: 0x07e7,
        char: 'η',
    },
    KeysymChar {
        keysym: 0x07e8,
        char: 'θ',
    },
    KeysymChar {
        keysym: 0x07e9,
        char: 'ι',
    },
    KeysymChar {
        keysym: 0x07ea,
        char: 'κ',
    },
    KeysymChar {
        keysym: 0x07eb,
        char: 'λ',
    },
    KeysymChar {
        keysym: 0x07ec,
        char: 'μ',
    },
    KeysymChar {
        keysym: 0x07ed,
        char: 'ν',
    },
    KeysymChar {
        keysym: 0x07ee,
        char: 'ξ',
    },
    KeysymChar {
        keysym: 0x07ef,
        char: 'ο',
    },
    KeysymChar {
        keysym: 0x07f0,
        char: 'π',
    },
    KeysymChar {
        keysym: 0x07f1,
        char: 'ρ',
    },
    KeysymChar {
        keysym: 0x07f2,
        char: 'σ',
    },
    KeysymChar {
        keysym: 0x07f3,
        char: 'ς',
    },
    KeysymChar {
        keysym: 0x07f4,
        char: 'τ',
    },
    KeysymChar {
        keysym: 0x07f5,
        char: 'υ',
    },
    KeysymChar {
        keysym: 0x07f6,
        char: 'φ',
    },
    KeysymChar {
        keysym: 0x07f7,
        char: 'χ',
    },
    KeysymChar {
        keysym: 0x07f8,
        char: 'ψ',
    },
    KeysymChar {
        keysym: 0x07f9,
        char: 'ω',
    },
    KeysymChar {
        keysym: 0x08a1,
        char: '⎷',
    },
    KeysymChar {
        keysym: 0x08a2,
        char: '┌',
    },
    KeysymChar {
        keysym: 0x08a3,
        char: '─',
    },
    KeysymChar {
        keysym: 0x08a4,
        char: '⌠',
    },
    KeysymChar {
        keysym: 0x08a5,
        char: '⌡',
    },
    KeysymChar {
        keysym: 0x08a6,
        char: '│',
    },
    KeysymChar {
        keysym: 0x08a7,
        char: '⎡',
    },
    KeysymChar {
        keysym: 0x08a8,
        char: '⎣',
    },
    KeysymChar {
        keysym: 0x08a9,
        char: '⎤',
    },
    KeysymChar {
        keysym: 0x08aa,
        char: '⎦',
    },
    KeysymChar {
        keysym: 0x08ab,
        char: '⎛',
    },
    KeysymChar {
        keysym: 0x08ac,
        char: '⎝',
    },
    KeysymChar {
        keysym: 0x08ad,
        char: '⎞',
    },
    KeysymChar {
        keysym: 0x08ae,
        char: '⎠',
    },
    KeysymChar {
        keysym: 0x08af,
        char: '⎨',
    },
    KeysymChar {
        keysym: 0x08b0,
        char: '⎬',
    },
    KeysymChar {
        keysym: 0x08bc,
        char: '≤',
    },
    KeysymChar {
        keysym: 0x08bd,
        char: '≠',
    },
    KeysymChar {
        keysym: 0x08be,
        char: '≥',
    },
    KeysymChar {
        keysym: 0x08bf,
        char: '∫',
    },
    KeysymChar {
        keysym: 0x08c0,
        char: '∴',
    },
    KeysymChar {
        keysym: 0x08c1,
        char: '∝',
    },
    KeysymChar {
        keysym: 0x08c2,
        char: '∞',
    },
    KeysymChar {
        keysym: 0x08c5,
        char: '∇',
    },
    KeysymChar {
        keysym: 0x08c8,
        char: '∼',
    },
    KeysymChar {
        keysym: 0x08c9,
        char: '≃',
    },
    KeysymChar {
        keysym: 0x08cd,
        char: '⇔',
    },
    KeysymChar {
        keysym: 0x08ce,
        char: '⇒',
    },
    KeysymChar {
        keysym: 0x08cf,
        char: '≡',
    },
    KeysymChar {
        keysym: 0x08d6,
        char: '√',
    },
    KeysymChar {
        keysym: 0x08da,
        char: '⊂',
    },
    KeysymChar {
        keysym: 0x08db,
        char: '⊃',
    },
    KeysymChar {
        keysym: 0x08dc,
        char: '∩',
    },
    KeysymChar {
        keysym: 0x08dd,
        char: '∪',
    },
    KeysymChar {
        keysym: 0x08de,
        char: '∧',
    },
    KeysymChar {
        keysym: 0x08df,
        char: '∨',
    },
    KeysymChar {
        keysym: 0x08ef,
        char: '∂',
    },
    KeysymChar {
        keysym: 0x08f6,
        char: 'ƒ',
    },
    KeysymChar {
        keysym: 0x08fb,
        char: '←',
    },
    KeysymChar {
        keysym: 0x08fc,
        char: '↑',
    },
    KeysymChar {
        keysym: 0x08fd,
        char: '→',
    },
    KeysymChar {
        keysym: 0x08fe,
        char: '↓',
    },
    KeysymChar {
        keysym: 0x09e0,
        char: '◆',
    },
    KeysymChar {
        keysym: 0x09e1,
        char: '▒',
    },
    KeysymChar {
        keysym: 0x09e2,
        char: '␉',
    },
    KeysymChar {
        keysym: 0x09e3,
        char: '␌',
    },
    KeysymChar {
        keysym: 0x09e4,
        char: '␍',
    },
    KeysymChar {
        keysym: 0x09e5,
        char: '␊',
    },
    KeysymChar {
        keysym: 0x09e8,
        char: '␤',
    },
    KeysymChar {
        keysym: 0x09e9,
        char: '␋',
    },
    KeysymChar {
        keysym: 0x09ea,
        char: '┘',
    },
    KeysymChar {
        keysym: 0x09eb,
        char: '┐',
    },
    KeysymChar {
        keysym: 0x09ec,
        char: '┌',
    },
    KeysymChar {
        keysym: 0x09ed,
        char: '└',
    },
    KeysymChar {
        keysym: 0x09ee,
        char: '┼',
    },
    KeysymChar {
        keysym: 0x09ef,
        char: '⎺',
    },
    KeysymChar {
        keysym: 0x09f0,
        char: '⎻',
    },
    KeysymChar {
        keysym: 0x09f1,
        char: '─',
    },
    KeysymChar {
        keysym: 0x09f2,
        char: '⎼',
    },
    KeysymChar {
        keysym: 0x09f3,
        char: '⎽',
    },
    KeysymChar {
        keysym: 0x09f4,
        char: '├',
    },
    KeysymChar {
        keysym: 0x09f5,
        char: '┤',
    },
    KeysymChar {
        keysym: 0x09f6,
        char: '┴',
    },
    KeysymChar {
        keysym: 0x09f7,
        char: '┬',
    },
    KeysymChar {
        keysym: 0x09f8,
        char: '│',
    },
    KeysymChar {
        keysym: 0x0aa1,
        char: '\u{2003}',
    },
    KeysymChar {
        keysym: 0x0aa2,
        char: '\u{2002}',
    },
    KeysymChar {
        keysym: 0x0aa3,
        char: '\u{2004}',
    },
    KeysymChar {
        keysym: 0x0aa4,
        char: '\u{2005}',
    },
    KeysymChar {
        keysym: 0x0aa5,
        char: '\u{2007}',
    },
    KeysymChar {
        keysym: 0x0aa6,
        char: '\u{2008}',
    },
    KeysymChar {
        keysym: 0x0aa7,
        char: '\u{2009}',
    },
    KeysymChar {
        keysym: 0x0aa8,
        char: '\u{200a}',
    },
    KeysymChar {
        keysym: 0x0aa9,
        char: '—',
    },
    KeysymChar {
        keysym: 0x0aaa,
        char: '–',
    },
    KeysymChar {
        keysym: 0x0aac,
        char: '␣',
    },
    KeysymChar {
        keysym: 0x0aae,
        char: '…',
    },
    KeysymChar {
        keysym: 0x0aaf,
        char: '‥',
    },
    KeysymChar {
        keysym: 0x0ab0,
        char: '⅓',
    },
    KeysymChar {
        keysym: 0x0ab1,
        char: '⅔',
    },
    KeysymChar {
        keysym: 0x0ab2,
        char: '⅕',
    },
    KeysymChar {
        keysym: 0x0ab3,
        char: '⅖',
    },
    KeysymChar {
        keysym: 0x0ab4,
        char: '⅗',
    },
    KeysymChar {
        keysym: 0x0ab5,
        char: '⅘',
    },
    KeysymChar {
        keysym: 0x0ab6,
        char: '⅙',
    },
    KeysymChar {
        keysym: 0x0ab7,
        char: '⅚',
    },
    KeysymChar {
        keysym: 0x0ab8,
        char: '℅',
    },
    KeysymChar {
        keysym: 0x0abb,
        char: '‒',
    },
    KeysymChar {
        keysym: 0x0abc,
        char: '⟨',
    },
    KeysymChar {
        keysym: 0x0abd,
        char: '.',
    },
    KeysymChar {
        keysym: 0x0abe,
        char: '⟩',
    },
    KeysymChar {
        keysym: 0x0ac3,
        char: '⅛',
    },
    KeysymChar {
        keysym: 0x0ac4,
        char: '⅜',
    },
    KeysymChar {
        keysym: 0x0ac5,
        char: '⅝',
    },
    KeysymChar {
        keysym: 0x0ac6,
        char: '⅞',
    },
    KeysymChar {
        keysym: 0x0ac9,
        char: '™',
    },
    KeysymChar {
        keysym: 0x0aca,
        char: '☓',
    },
    KeysymChar {
        keysym: 0x0acc,
        char: '◁',
    },
    KeysymChar {
        keysym: 0x0acd,
        char: '▷',
    },
    KeysymChar {
        keysym: 0x0ace,
        char: '○',
    },
    KeysymChar {
        keysym: 0x0acf,
        char: '▯',
    },
    KeysymChar {
        keysym: 0x0ad0,
        char: '‘',
    },
    KeysymChar {
        keysym: 0x0ad1,
        char: '’',
    },
    KeysymChar {
        keysym: 0x0ad2,
        char: '“',
    },
    KeysymChar {
        keysym: 0x0ad3,
        char: '”',
    },
    KeysymChar {
        keysym: 0x0ad4,
        char: '℞',
    },
    KeysymChar {
        keysym: 0x0ad5,
        char: '‰',
    },
    KeysymChar {
        keysym: 0x0ad6,
        char: '′',
    },
    KeysymChar {
        keysym: 0x0ad7,
        char: '″',
    },
    KeysymChar {
        keysym: 0x0ad9,
        char: '✝',
    },
    KeysymChar {
        keysym: 0x0adb,
        char: '▬',
    },
    KeysymChar {
        keysym: 0x0adc,
        char: '◀',
    },
    KeysymChar {
        keysym: 0x0add,
        char: '▶',
    },
    KeysymChar {
        keysym: 0x0ade,
        char: '●',
    },
    KeysymChar {
        keysym: 0x0adf,
        char: '▮',
    },
    KeysymChar {
        keysym: 0x0ae0,
        char: '◦',
    },
    KeysymChar {
        keysym: 0x0ae1,
        char: '▫',
    },
    KeysymChar {
        keysym: 0x0ae2,
        char: '▭',
    },
    KeysymChar {
        keysym: 0x0ae3,
        char: '△',
    },
    KeysymChar {
        keysym: 0x0ae4,
        char: '▽',
    },
    KeysymChar {
        keysym: 0x0ae5,
        char: '☆',
    },
    KeysymChar {
        keysym: 0x0ae6,
        char: '•',
    },
    KeysymChar {
        keysym: 0x0ae7,
        char: '▪',
    },
    KeysymChar {
        keysym: 0x0ae8,
        char: '▲',
    },
    KeysymChar {
        keysym: 0x0ae9,
        char: '▼',
    },
    KeysymChar {
        keysym: 0x0aea,
        char: '☜',
    },
    KeysymChar {
        keysym: 0x0aeb,
        char: '☞',
    },
    KeysymChar {
        keysym: 0x0aec,
        char: '♣',
    },
    KeysymChar {
        keysym: 0x0aed,
        char: '♦',
    },
    KeysymChar {
        keysym: 0x0aee,
        char: '♥',
    },
    KeysymChar {
        keysym: 0x0af0,
        char: '✠',
    },
    KeysymChar {
        keysym: 0x0af1,
        char: '†',
    },
    KeysymChar {
        keysym: 0x0af2,
        char: '‡',
    },
    KeysymChar {
        keysym: 0x0af3,
        char: '✓',
    },
    KeysymChar {
        keysym: 0x0af4,
        char: '✗',
    },
    KeysymChar {
        keysym: 0x0af5,
        char: '♯',
    },
    KeysymChar {
        keysym: 0x0af6,
        char: '♭',
    },
    KeysymChar {
        keysym: 0x0af7,
        char: '♂',
    },
    KeysymChar {
        keysym: 0x0af8,
        char: '♀',
    },
    KeysymChar {
        keysym: 0x0af9,
        char: '☎',
    },
    KeysymChar {
        keysym: 0x0afa,
        char: '⌕',
    },
    KeysymChar {
        keysym: 0x0afb,
        char: '℗',
    },
    KeysymChar {
        keysym: 0x0afc,
        char: '‸',
    },
    KeysymChar {
        keysym: 0x0afd,
        char: '‚',
    },
    KeysymChar {
        keysym: 0x0afe,
        char: '„',
    },
    KeysymChar {
        keysym: 0x0ba3,
        char: '<',
    },
    KeysymChar {
        keysym: 0x0ba6,
        char: '>',
    },
    KeysymChar {
        keysym: 0x0ba8,
        char: '∨',
    },
    KeysymChar {
        keysym: 0x0ba9,
        char: '∧',
    },
    KeysymChar {
        keysym: 0x0bc0,
        char: '¯',
    },
    KeysymChar {
        keysym: 0x0bc2,
        char: '⊤',
    },
    KeysymChar {
        keysym: 0x0bc3,
        char: '∩',
    },
    KeysymChar {
        keysym: 0x0bc4,
        char: '⌊',
    },
    KeysymChar {
        keysym: 0x0bc6,
        char: '_',
    },
    KeysymChar {
        keysym: 0x0bca,
        char: '∘',
    },
    KeysymChar {
        keysym: 0x0bcc,
        char: '⎕',
    },
    KeysymChar {
        keysym: 0x0bce,
        char: '⊥',
    },
    KeysymChar {
        keysym: 0x0bcf,
        char: '○',
    },
    KeysymChar {
        keysym: 0x0bd3,
        char: '⌈',
    },
    KeysymChar {
        keysym: 0x0bd6,
        char: '∪',
    },
    KeysymChar {
        keysym: 0x0bd8,
        char: '⊃',
    },
    KeysymChar {
        keysym: 0x0bda,
        char: '⊂',
    },
    KeysymChar {
        keysym: 0x0bdc,
        char: '⊣',
    },
    KeysymChar {
        keysym: 0x0bfc,
        char: '⊢',
    },
    KeysymChar {
        keysym: 0x0cdf,
        char: '‗',
    },
    KeysymChar {
        keysym: 0x0ce0,
        char: 'א',
    },
    KeysymChar {
        keysym: 0x0ce1,
        char: 'ב',
    },
    KeysymChar {
        keysym: 0x0ce2,
        char: 'ג',
    },
    KeysymChar {
        keysym: 0x0ce3,
        char: 'ד',
    },
    KeysymChar {
        keysym: 0x0ce4,
        char: 'ה',
    },
    KeysymChar {
        keysym: 0x0ce5,
        char: 'ו',
    },
    KeysymChar {
        keysym: 0x0ce6,
        char: 'ז',
    },
    KeysymChar {
        keysym: 0x0ce7,
        char: 'ח',
    },
    KeysymChar {
        keysym: 0x0ce8,
        char: 'ט',
    },
    KeysymChar {
        keysym: 0x0ce9,
        char: 'י',
    },
    KeysymChar {
        keysym: 0x0cea,
        char: 'ך',
    },
    KeysymChar {
        keysym: 0x0ceb,
        char: 'כ',
    },
    KeysymChar {
        keysym: 0x0cec,
        char: 'ל',
    },
    KeysymChar {
        keysym: 0x0ced,
        char: 'ם',
    },
    KeysymChar {
        keysym: 0x0cee,
        char: 'מ',
    },
    KeysymChar {
        keysym: 0x0cef,
        char: 'ן',
    },
    KeysymChar {
        keysym: 0x0cf0,
        char: 'נ',
    },
    KeysymChar {
        keysym: 0x0cf1,
        char: 'ס',
    },
    KeysymChar {
        keysym: 0x0cf2,
        char: 'ע',
    },
    KeysymChar {
        keysym: 0x0cf3,
        char: 'ף',
    },
    KeysymChar {
        keysym: 0x0cf4,
        char: 'פ',
    },
    KeysymChar {
        keysym: 0x0cf5,
        char: 'ץ',
    },
    KeysymChar {
        keysym: 0x0cf6,
        char: 'צ',
    },
    KeysymChar {
        keysym: 0x0cf7,
        char: 'ק',
    },
    KeysymChar {
        keysym: 0x0cf8,
        char: 'ר',
    },
    KeysymChar {
        keysym: 0x0cf9,
        char: 'ש',
    },
    KeysymChar {
        keysym: 0x0cfa,
        char: 'ת',
    },
    KeysymChar {
        keysym: 0x0da1,
        char: 'ก',
    },
    KeysymChar {
        keysym: 0x0da2,
        char: 'ข',
    },
    KeysymChar {
        keysym: 0x0da3,
        char: 'ฃ',
    },
    KeysymChar {
        keysym: 0x0da4,
        char: 'ค',
    },
    KeysymChar {
        keysym: 0x0da5,
        char: 'ฅ',
    },
    KeysymChar {
        keysym: 0x0da6,
        char: 'ฆ',
    },
    KeysymChar {
        keysym: 0x0da7,
        char: 'ง',
    },
    KeysymChar {
        keysym: 0x0da8,
        char: 'จ',
    },
    KeysymChar {
        keysym: 0x0da9,
        char: 'ฉ',
    },
    KeysymChar {
        keysym: 0x0daa,
        char: 'ช',
    },
    KeysymChar {
        keysym: 0x0dab,
        char: 'ซ',
    },
    KeysymChar {
        keysym: 0x0dac,
        char: 'ฌ',
    },
    KeysymChar {
        keysym: 0x0dad,
        char: 'ญ',
    },
    KeysymChar {
        keysym: 0x0dae,
        char: 'ฎ',
    },
    KeysymChar {
        keysym: 0x0daf,
        char: 'ฏ',
    },
    KeysymChar {
        keysym: 0x0db0,
        char: 'ฐ',
    },
    KeysymChar {
        keysym: 0x0db1,
        char: 'ฑ',
    },
    KeysymChar {
        keysym: 0x0db2,
        char: 'ฒ',
    },
    KeysymChar {
        keysym: 0x0db3,
        char: 'ณ',
    },
    KeysymChar {
        keysym: 0x0db4,
        char: 'ด',
    },
    KeysymChar {
        keysym: 0x0db5,
        char: 'ต',
    },
    KeysymChar {
        keysym: 0x0db6,
        char: 'ถ',
    },
    KeysymChar {
        keysym: 0x0db7,
        char: 'ท',
    },
    KeysymChar {
        keysym: 0x0db8,
        char: 'ธ',
    },
    KeysymChar {
        keysym: 0x0db9,
        char: 'น',
    },
    KeysymChar {
        keysym: 0x0dba,
        char: 'บ',
    },
    KeysymChar {
        keysym: 0x0dbb,
        char: 'ป',
    },
    KeysymChar {
        keysym: 0x0dbc,
        char: 'ผ',
    },
    KeysymChar {
        keysym: 0x0dbd,
        char: 'ฝ',
    },
    KeysymChar {
        keysym: 0x0dbe,
        char: 'พ',
    },
    KeysymChar {
        keysym: 0x0dbf,
        char: 'ฟ',
    },
    KeysymChar {
        keysym: 0x0dc0,
        char: 'ภ',
    },
    KeysymChar {
        keysym: 0x0dc1,
        char: 'ม',
    },
    KeysymChar {
        keysym: 0x0dc2,
        char: 'ย',
    },
    KeysymChar {
        keysym: 0x0dc3,
        char: 'ร',
    },
    KeysymChar {
        keysym: 0x0dc4,
        char: 'ฤ',
    },
    KeysymChar {
        keysym: 0x0dc5,
        char: 'ล',
    },
    KeysymChar {
        keysym: 0x0dc6,
        char: 'ฦ',
    },
    KeysymChar {
        keysym: 0x0dc7,
        char: 'ว',
    },
    KeysymChar {
        keysym: 0x0dc8,
        char: 'ศ',
    },
    KeysymChar {
        keysym: 0x0dc9,
        char: 'ษ',
    },
    KeysymChar {
        keysym: 0x0dca,
        char: 'ส',
    },
    KeysymChar {
        keysym: 0x0dcb,
        char: 'ห',
    },
    KeysymChar {
        keysym: 0x0dcc,
        char: 'ฬ',
    },
    KeysymChar {
        keysym: 0x0dcd,
        char: 'อ',
    },
    KeysymChar {
        keysym: 0x0dce,
        char: 'ฮ',
    },
    KeysymChar {
        keysym: 0x0dcf,
        char: 'ฯ',
    },
    KeysymChar {
        keysym: 0x0dd0,
        char: 'ะ',
    },
    KeysymChar {
        keysym: 0x0dd1,
        char: '\u{e31}',
    },
    KeysymChar {
        keysym: 0x0dd2,
        char: 'า',
    },
    KeysymChar {
        keysym: 0x0dd3,
        char: 'ำ',
    },
    KeysymChar {
        keysym: 0x0dd4,
        char: '\u{e34}',
    },
    KeysymChar {
        keysym: 0x0dd5,
        char: '\u{e35}',
    },
    KeysymChar {
        keysym: 0x0dd6,
        char: '\u{e36}',
    },
    KeysymChar {
        keysym: 0x0dd7,
        char: '\u{e37}',
    },
    KeysymChar {
        keysym: 0x0dd8,
        char: '\u{e38}',
    },
    KeysymChar {
        keysym: 0x0dd9,
        char: '\u{e39}',
    },
    KeysymChar {
        keysym: 0x0dda,
        char: '\u{e3a}',
    },
    KeysymChar {
        keysym: 0x0dde,
        char: '\u{e3e}',
    },
    KeysymChar {
        keysym: 0x0ddf,
        char: '฿',
    },
    KeysymChar {
        keysym: 0x0de0,
        char: 'เ',
    },
    KeysymChar {
        keysym: 0x0de1,
        char: 'แ',
    },
    KeysymChar {
        keysym: 0x0de2,
        char: 'โ',
    },
    KeysymChar {
        keysym: 0x0de3,
        char: 'ใ',
    },
    KeysymChar {
        keysym: 0x0de4,
        char: 'ไ',
    },
    KeysymChar {
        keysym: 0x0de5,
        char: 'ๅ',
    },
    KeysymChar {
        keysym: 0x0de6,
        char: 'ๆ',
    },
    KeysymChar {
        keysym: 0x0de7,
        char: '\u{e47}',
    },
    KeysymChar {
        keysym: 0x0de8,
        char: '\u{e48}',
    },
    KeysymChar {
        keysym: 0x0de9,
        char: '\u{e49}',
    },
    KeysymChar {
        keysym: 0x0dea,
        char: '\u{e4a}',
    },
    KeysymChar {
        keysym: 0x0deb,
        char: '\u{e4b}',
    },
    KeysymChar {
        keysym: 0x0dec,
        char: '\u{e4c}',
    },
    KeysymChar {
        keysym: 0x0ded,
        char: '\u{e4d}',
    },
    KeysymChar {
        keysym: 0x0df0,
        char: '๐',
    },
    KeysymChar {
        keysym: 0x0df1,
        char: '๑',
    },
    KeysymChar {
        keysym: 0x0df2,
        char: '๒',
    },
    KeysymChar {
        keysym: 0x0df3,
        char: '๓',
    },
    KeysymChar {
        keysym: 0x0df4,
        char: '๔',
    },
    KeysymChar {
        keysym: 0x0df5,
        char: '๕',
    },
    KeysymChar {
        keysym: 0x0df6,
        char: '๖',
    },
    KeysymChar {
        keysym: 0x0df7,
        char: '๗',
    },
    KeysymChar {
        keysym: 0x0df8,
        char: '๘',
    },
    KeysymChar {
        keysym: 0x0df9,
        char: '๙',
    },
    KeysymChar {
        keysym: 0x0ea1,
        char: 'ㄱ',
    },
    KeysymChar {
        keysym: 0x0ea2,
        char: 'ㄲ',
    },
    KeysymChar {
        keysym: 0x0ea3,
        char: 'ㄳ',
    },
    KeysymChar {
        keysym: 0x0ea4,
        char: 'ㄴ',
    },
    KeysymChar {
        keysym: 0x0ea5,
        char: 'ㄵ',
    },
    KeysymChar {
        keysym: 0x0ea6,
        char: 'ㄶ',
    },
    KeysymChar {
        keysym: 0x0ea7,
        char: 'ㄷ',
    },
    KeysymChar {
        keysym: 0x0ea8,
        char: 'ㄸ',
    },
    KeysymChar {
        keysym: 0x0ea9,
        char: 'ㄹ',
    },
    KeysymChar {
        keysym: 0x0eaa,
        char: 'ㄺ',
    },
    KeysymChar {
        keysym: 0x0eab,
        char: 'ㄻ',
    },
    KeysymChar {
        keysym: 0x0eac,
        char: 'ㄼ',
    },
    KeysymChar {
        keysym: 0x0ead,
        char: 'ㄽ',
    },
    KeysymChar {
        keysym: 0x0eae,
        char: 'ㄾ',
    },
    KeysymChar {
        keysym: 0x0eaf,
        char: 'ㄿ',
    },
    KeysymChar {
        keysym: 0x0eb0,
        char: 'ㅀ',
    },
    KeysymChar {
        keysym: 0x0eb1,
        char: 'ㅁ',
    },
    KeysymChar {
        keysym: 0x0eb2,
        char: 'ㅂ',
    },
    KeysymChar {
        keysym: 0x0eb3,
        char: 'ㅃ',
    },
    KeysymChar {
        keysym: 0x0eb4,
        char: 'ㅄ',
    },
    KeysymChar {
        keysym: 0x0eb5,
        char: 'ㅅ',
    },
    KeysymChar {
        keysym: 0x0eb6,
        char: 'ㅆ',
    },
    KeysymChar {
        keysym: 0x0eb7,
        char: 'ㅇ',
    },
    KeysymChar {
        keysym: 0x0eb8,
        char: 'ㅈ',
    },
    KeysymChar {
        keysym: 0x0eb9,
        char: 'ㅉ',
    },
    KeysymChar {
        keysym: 0x0eba,
        char: 'ㅊ',
    },
    KeysymChar {
        keysym: 0x0ebb,
        char: 'ㅋ',
    },
    KeysymChar {
        keysym: 0x0ebc,
        char: 'ㅌ',
    },
    KeysymChar {
        keysym: 0x0ebd,
        char: 'ㅍ',
    },
    KeysymChar {
        keysym: 0x0ebe,
        char: 'ㅎ',
    },
    KeysymChar {
        keysym: 0x0ebf,
        char: 'ㅏ',
    },
    KeysymChar {
        keysym: 0x0ec0,
        char: 'ㅐ',
    },
    KeysymChar {
        keysym: 0x0ec1,
        char: 'ㅑ',
    },
    KeysymChar {
        keysym: 0x0ec2,
        char: 'ㅒ',
    },
    KeysymChar {
        keysym: 0x0ec3,
        char: 'ㅓ',
    },
    KeysymChar {
        keysym: 0x0ec4,
        char: 'ㅔ',
    },
    KeysymChar {
        keysym: 0x0ec5,
        char: 'ㅕ',
    },
    KeysymChar {
        keysym: 0x0ec6,
        char: 'ㅖ',
    },
    KeysymChar {
        keysym: 0x0ec7,
        char: 'ㅗ',
    },
    KeysymChar {
        keysym: 0x0ec8,
        char: 'ㅘ',
    },
    KeysymChar {
        keysym: 0x0ec9,
        char: 'ㅙ',
    },
    KeysymChar {
        keysym: 0x0eca,
        char: 'ㅚ',
    },
    KeysymChar {
        keysym: 0x0ecb,
        char: 'ㅛ',
    },
    KeysymChar {
        keysym: 0x0ecc,
        char: 'ㅜ',
    },
    KeysymChar {
        keysym: 0x0ecd,
        char: 'ㅝ',
    },
    KeysymChar {
        keysym: 0x0ece,
        char: 'ㅞ',
    },
    KeysymChar {
        keysym: 0x0ecf,
        char: 'ㅟ',
    },
    KeysymChar {
        keysym: 0x0ed0,
        char: 'ㅠ',
    },
    KeysymChar {
        keysym: 0x0ed1,
        char: 'ㅡ',
    },
    KeysymChar {
        keysym: 0x0ed2,
        char: 'ㅢ',
    },
    KeysymChar {
        keysym: 0x0ed3,
        char: 'ㅣ',
    },
    KeysymChar {
        keysym: 0x0ed4,
        char: 'ᆨ',
    },
    KeysymChar {
        keysym: 0x0ed5,
        char: 'ᆩ',
    },
    KeysymChar {
        keysym: 0x0ed6,
        char: 'ᆪ',
    },
    KeysymChar {
        keysym: 0x0ed7,
        char: 'ᆫ',
    },
    KeysymChar {
        keysym: 0x0ed8,
        char: 'ᆬ',
    },
    KeysymChar {
        keysym: 0x0ed9,
        char: 'ᆭ',
    },
    KeysymChar {
        keysym: 0x0eda,
        char: 'ᆮ',
    },
    KeysymChar {
        keysym: 0x0edb,
        char: 'ᆯ',
    },
    KeysymChar {
        keysym: 0x0edc,
        char: 'ᆰ',
    },
    KeysymChar {
        keysym: 0x0edd,
        char: 'ᆱ',
    },
    KeysymChar {
        keysym: 0x0ede,
        char: 'ᆲ',
    },
    KeysymChar {
        keysym: 0x0edf,
        char: 'ᆳ',
    },
    KeysymChar {
        keysym: 0x0ee0,
        char: 'ᆴ',
    },
    KeysymChar {
        keysym: 0x0ee1,
        char: 'ᆵ',
    },
    KeysymChar {
        keysym: 0x0ee2,
        char: 'ᆶ',
    },
    KeysymChar {
        keysym: 0x0ee3,
        char: 'ᆷ',
    },
    KeysymChar {
        keysym: 0x0ee4,
        char: 'ᆸ',
    },
    KeysymChar {
        keysym: 0x0ee5,
        char: 'ᆹ',
    },
    KeysymChar {
        keysym: 0x0ee6,
        char: 'ᆺ',
    },
    KeysymChar {
        keysym: 0x0ee7,
        char: 'ᆻ',
    },
    KeysymChar {
        keysym: 0x0ee8,
        char: 'ᆼ',
    },
    KeysymChar {
        keysym: 0x0ee9,
        char: 'ᆽ',
    },
    KeysymChar {
        keysym: 0x0eea,
        char: 'ᆾ',
    },
    KeysymChar {
        keysym: 0x0eeb,
        char: 'ᆿ',
    },
    KeysymChar {
        keysym: 0x0eec,
        char: 'ᇀ',
    },
    KeysymChar {
        keysym: 0x0eed,
        char: 'ᇁ',
    },
    KeysymChar {
        keysym: 0x0eee,
        char: 'ᇂ',
    },
    KeysymChar {
        keysym: 0x0eef,
        char: 'ㅭ',
    },
    KeysymChar {
        keysym: 0x0ef0,
        char: 'ㅱ',
    },
    KeysymChar {
        keysym: 0x0ef1,
        char: 'ㅸ',
    },
    KeysymChar {
        keysym: 0x0ef2,
        char: 'ㅿ',
    },
    KeysymChar {
        keysym: 0x0ef3,
        char: 'ㆁ',
    },
    KeysymChar {
        keysym: 0x0ef4,
        char: 'ㆄ',
    },
    KeysymChar {
        keysym: 0x0ef5,
        char: 'ㆆ',
    },
    KeysymChar {
        keysym: 0x0ef6,
        char: 'ㆍ',
    },
    KeysymChar {
        keysym: 0x0ef7,
        char: 'ㆎ',
    },
    KeysymChar {
        keysym: 0x0ef8,
        char: 'ᇫ',
    },
    KeysymChar {
        keysym: 0x0ef9,
        char: 'ᇰ',
    },
    KeysymChar {
        keysym: 0x0efa,
        char: 'ᇹ',
    },
    KeysymChar {
        keysym: 0x0eff,
        char: '₩',
    },
    KeysymChar {
        keysym: 0x13bc,
        char: 'Œ',
    },
    KeysymChar {
        keysym: 0x13bd,
        char: 'œ',
    },
    KeysymChar {
        keysym: 0x13be,
        char: 'Ÿ',
    },
    KeysymChar {
        keysym: 0x20ac,
        char: '€',
    },
    KeysymChar {
        keysym: 0xff08,
        char: '\u{8}',
    },
    KeysymChar {
        keysym: 0xff09,
        char: '\t',
    },
    KeysymChar {
        keysym: 0xff0a,
        char: '\n',
    },
    KeysymChar {
        keysym: 0xff0b,
        char: '\u{b}',
    },
    KeysymChar {
        keysym: 0xff0d,
        char: '\r',
    },
    KeysymChar {
        keysym: 0xff1b,
        char: '\u{1b}',
    },
    KeysymChar {
        keysym: 0xff80,
        char: ' ',
    },
    KeysymChar {
        keysym: 0xff89,
        char: '\t',
    },
    KeysymChar {
        keysym: 0xff8d,
        char: '\r',
    },
    KeysymChar {
        keysym: 0xffaa,
        char: '*',
    },
    KeysymChar {
        keysym: 0xffab,
        char: '+',
    },
    KeysymChar {
        keysym: 0xffac,
        char: ',',
    },
    KeysymChar {
        keysym: 0xffad,
        char: '-',
    },
    KeysymChar {
        keysym: 0xffae,
        char: '.',
    },
    KeysymChar {
        keysym: 0xffaf,
        char: '/',
    },
    KeysymChar {
        keysym: 0xffb0,
        char: '0',
    },
    KeysymChar {
        keysym: 0xffb1,
        char: '1',
    },
    KeysymChar {
        keysym: 0xffb2,
        char: '2',
    },
    KeysymChar {
        keysym: 0xffb3,
        char: '3',
    },
    KeysymChar {
        keysym: 0xffb4,
        char: '4',
    },
    KeysymChar {
        keysym: 0xffb5,
        char: '5',
    },
    KeysymChar {
        keysym: 0xffb6,
        char: '6',
    },
    KeysymChar {
        keysym: 0xffb7,
        char: '7',
    },
    KeysymChar {
        keysym: 0xffb8,
        char: '8',
    },
    KeysymChar {
        keysym: 0xffb9,
        char: '9',
    },
    KeysymChar {
        keysym: 0xffbd,
        char: '=',
    },
    KeysymChar {
        keysym: 0xffff,
        char: '\u{7f}',
    },
];

pub(super) static CHAR_TO_BESPOKE_IDX: IndexMap<char, u16> = IndexMap {
    key: 12913932095322966823,
    disps: &[(0, 201), (0, 13), (0, 0), (0, 46), (0, 96), (0, 0), (0, 56), (0, 30), (0, 7), (0, 205), (0, 116), (0, 4), (0, 47), (0, 3), (0, 7), (0, 0), (0, 268), (0, 100), (0, 0), (0, 34), (0, 306), (0, 256), (0, 21), (0, 176), (0, 17), (0, 0), (0, 10), (0, 0), (0, 4), (0, 0), (0, 1), (0, 806), (0, 1), (0, 0), (0, 93), (0, 1), (0, 0), (0, 0), (0, 0), (0, 1), (0, 28), (0, 0), (0, 0), (0, 0), (0, 0), (0, 96), (0, 1), (0, 1), (0, 43), (0, 36), (0, 767), (0, 7), (0, 52), (0, 0), (0, 3), (0, 24), (0, 8), (0, 62), (0, 0), (0, 11), (0, 73), (0, 0), (0, 2), (0, 167), (0, 0), (0, 0), (0, 71), (0, 5), (0, 34), (0, 92), (0, 16), (0, 0), (0, 62), (0, 175), (0, 1), (0, 110), (0, 116), (0, 0), (0, 58), (0, 2), (0, 2), (0, 4), (0, 11), (0, 7), (0, 0), (0, 4), (0, 31), (0, 3), (0, 176), (0, 34), (0, 25), (0, 0), (0, 27), (0, 10), (0, 469), (0, 19), (0, 8), (0, 792), (0, 5), (0, 35), (0, 10), (0, 4), (0, 229), (0, 132), (0, 15), (0, 0), (0, 85), (0, 151), (0, 16), (0, 0), (0, 1), (0, 2), (0, 0), (0, 1), (0, 12), (0, 4), (0, 47), (0, 6), (0, 22), (0, 8), (0, 211), (0, 0), (0, 12), (0, 0), (0, 0), (0, 50), (0, 0), (0, 6), (0, 3), (0, 5), (0, 7), (0, 221), (0, 0), (0, 98), (0, 143), (0, 37), (0, 46), (0, 377), (0, 15), (0, 15), (0, 30), (0, 9), (0, 17), (0, 43), (0, 30), (0, 415), (0, 4), (0, 4), (0, 18), (0, 0), (0, 5), (0, 99), (0, 445), (0, 152), (0, 33), (0, 162), (0, 255), (0, 578), (0, 218), (0, 163), (1, 459), (0, 7), (0, 0), (2, 192), (0, 6), (0, 67), (0, 56), (0, 22), (0, 6), (0, 428), (0, 16), (0, 5), (0, 146), (0, 731), (0, 23), (0, 6), (0, 10), (0, 414), (0, 107), (0, 32), (0, 145), (0, 90), (0, 237), (0, 7), (0, 4), (0, 151), (0, 7), (0, 507), (0, 432), (0, 36), (0, 75), (0, 88), (0, 51), (0, 326), (0, 17), (0, 258), (0, 3), (0, 238), (0, 0), (0, 4), (0, 0), (0, 161), (0, 1), (0, 37), (0, 4), (0, 120), (0, 4), (0, 8), (0, 3), (0, 0), (0, 698), (0, 7), (0, 0), (0, 17), (0, 164), (0, 48), (1, 258), (0, 2), (0, 220), (0, 553), (0, 857), (0, 177), (0, 9), (1, 66), (4, 46), (0, 240), (1, 505), (0, 46), (0, 876), (0, 1), (0, 14), (2, 421), (0, 320), (2, 426), (1, 69), (0, 33), (0, 17), (2, 108), (0, 2), (0, 1), (0, 14), (1, 634), (0, 10), (2, 579), (0, 266), (0, 131), (0, 8), (2, 357), (0, 175), (1, 87), (0, 184), (0, 604), (0, 65), (4, 712), (0, 1), (0, 12)],
    map: &[661, 1112, 589, 1115, 710, 1314, 1339, 821, 902, 751, 1260, 1021, 522, 1182, 1020, 1375, 478, 1410, 106, 469, 714, 1209, 1443, 588, 1305, 1152, 432, 728, 817, 435, 491, 998, 461, 600, 752, 1022, 1352, 1126, 958, 1378, 1239, 989, 585, 631, 412, 1226, 742, 645, 953, 1160, 538, 755, 528, 1038, 1153, 1427, 400, 1365, 1083, 488, 773, 1463, 452, 812, 1199, 1142, 834, 1277, 1422, 762, 662, 1306, 403, 489, 562, 766, 1139, 644, 611, 1029, 854, 1178, 429, 1358, 636, 1439, 1071, 961, 103, 1330, 976, 1449, 374, 1441, 1058, 444, 555, 404, 1345, 733, 582, 768, 670, 1087, 1336, 487, 1003, 640, 835, 923, 987, 568, 1093, 679, 1479, 115, 513, 535, 947, 1192, 855, 601, 1362, 397, 676, 1035, 581, 911, 1155, 618, 973, 1407, 1248, 664, 907, 983, 1082, 1288, 510, 524, 406, 1259, 532, 505, 1122, 959, 421, 1451, 479, 851, 1323, 1406, 371, 1215, 1459, 624, 552, 1047, 1436, 592, 1315, 1377, 498, 1050, 827, 1127, 1234, 1236, 1333, 6, 673, 620, 1411, 1005, 654, 1344, 1194, 500, 373, 1354, 1177, 112, 818, 1173, 943, 913, 964, 1227, 1123, 440, 613, 926, 723, 518, 749, 652, 1311, 1398, 774, 1166, 1404, 669, 1468, 436, 420, 657, 1446, 738, 470, 418, 1428, 1032, 1229, 956, 544, 457, 1370, 612, 1307, 839, 924, 1371, 760, 543, 903, 502, 727, 619, 1350, 497, 927, 458, 1084, 1328, 637, 628, 1001, 1107, 520, 104, 494, 467, 580, 1059, 1364, 711, 1197, 1303, 1460, 647, 732, 938, 622, 815, 460, 1205, 1014, 2, 1442, 717, 1158, 1143, 1080, 929, 974, 771, 1272, 1157, 621, 741, 948, 1270, 536, 730, 1480, 561, 747, 836, 985, 1073, 632, 1201, 1214, 1054, 941, 1320, 101, 1368, 1131, 1114, 990, 569, 511, 1274, 1045, 1363, 832, 1297, 372, 427, 1416, 1342, 1119, 759, 1289, 1425, 441, 1437, 1473, 579, 1151, 614, 593, 553, 506, 485, 1224, 1268, 1068, 606, 1057, 920, 1457, 1191, 1419, 402, 660, 1478, 1007, 737, 1135, 530, 811, 1334, 1092, 1016, 629, 442, 756, 1417, 572, 819, 571, 753, 1134, 395, 549, 1312, 1216, 1360, 658, 996, 824, 1453, 1475, 603, 852, 1261, 113, 1405, 1286, 778, 986, 419, 369, 1064, 954, 1168, 413, 1065, 508, 577, 507, 849, 1175, 610, 459, 605, 842, 665, 1202, 709, 1329, 1116, 1484, 944, 722, 1396, 1420, 570, 1203, 1254, 575, 643, 1331, 671, 1353, 541, 1212, 1409, 1012, 1099, 1481, 527, 455, 1304, 468, 946, 816, 433, 1482, 110, 1397, 1235, 1309, 1081, 979, 411, 1280, 1121, 1343, 940, 1146, 772, 1232, 1167, 1132, 438, 1025, 1348, 1282, 450, 1426, 430, 932, 1174, 451, 1138, 1055, 1042, 659, 558, 486, 833, 410, 968, 921, 1444, 1470, 1128, 1223, 1474, 1321, 763, 978, 1399, 653, 731, 963, 1461, 726, 426, 102, 1154, 1163, 1217, 1326, 378, 1111, 1078, 1456, 677, 813, 609, 1200, 1369, 1233, 496, 971, 767, 1293, 1316, 1228, 840, 424, 428, 1322, 1402, 1458, 1269, 740, 1418, 962, 1129, 1033, 666, 713, 1186, 586, 1034, 396, 1252, 464, 934, 909, 735, 655, 1318, 1221, 1118, 830, 1052, 981, 1351, 758, 1301, 988, 423, 1414, 1295, 1222, 1340, 550, 918, 1380, 1066, 1075, 853, 744, 1435, 625, 1120, 1361, 663, 551, 1049, 841, 439, 1434, 1455, 1171, 754, 1264, 476, 509, 604, 1230, 1381, 1376, 587, 972, 1208, 1332, 1477, 641, 1013, 542, 1431, 845, 1165, 639, 547, 111, 736, 456, 1048, 1310, 1467, 116, 1281, 980, 1156, 822, 1113, 1253, 1056, 596, 1283, 1140, 474, 417, 1062, 1450, 1164, 1213, 523, 107, 850, 957, 672, 847, 955, 638, 1447, 1394, 578, 501, 967, 567, 616, 1010, 1349, 721, 1471, 1327, 1098, 1028, 3, 1095, 1466, 1063, 770, 1161, 519, 1079, 999, 1476, 448, 407, 734, 1211, 1067, 539, 602, 950, 416, 1237, 1039, 1366, 108, 642, 1026, 1189, 1464, 798, 668, 565, 1462, 814, 1452, 1395, 1072, 1141, 674, 375, 970, 607, 514, 1278, 949, 1117, 1130, 1423, 483, 1187, 449, 627, 1341, 1198, 1219, 1053, 1440, 1319, 651, 484, 743, 1251, 904, 630, 942, 646, 1324, 1088, 725, 466, 1337, 1179, 831, 534, 1000, 1019, 1258, 810, 1108, 1296, 1415, 445, 1002, 1210, 1195, 11, 952, 480, 617, 1266, 1150, 1367, 425, 1023, 1169, 1408, 712, 1382, 1225, 409, 1238, 556, 1206, 682, 521, 993, 1287, 1432, 745, 838, 739, 548, 1483, 1184, 960, 1036, 1359, 117, 823, 499, 453, 422, 1299, 503, 764, 377, 492, 828, 1220, 984, 1284, 779, 1077, 1379, 462, 608, 81, 1061, 437, 1412, 1124, 1741, 591, 594, 1355, 777, 848, 1448, 1027, 1250, 1172, 1433, 370, 1031, 1403, 599, 4, 414, 1096, 1024, 965, 471, 915, 1148, 650, 623, 1424, 843, 775, 540, 634, 1231, 525, 1097, 1429, 1188, 109, 447, 590, 1302, 598, 678, 1308, 635, 545, 1465, 1170, 1472, 1372, 1338, 1279, 1145, 1137, 454, 1401, 951, 680, 1046, 515, 1180, 431, 626, 1109, 105, 1149, 1011, 401, 930, 557, 576, 10, 408, 595, 901, 995, 1162, 1144, 994, 1347, 1018, 769, 1041, 919, 405, 837, 1094, 1325, 856, 516, 1006, 1246, 1076, 1421, 1159, 969, 573, 537, 975, 446, 1196, 1017, 1300, 399, 667, 1043, 715, 720, 559, 574, 857, 1091, 724, 490, 1276, 563, 475, 1037, 757, 991, 977, 992, 1136, 481, 936, 1291, 716, 1373, 1185, 1357, 1004, 1469, 1438, 465, 1070, 1040, 1445, 554, 1051, 820, 765, 1317, 675, 780, 564, 482, 829, 649, 583, 997, 5, 1413, 1335, 917, 566, 597, 1009, 633, 906, 1356, 512, 531, 1110, 1147, 656, 1346, 1454, 463, 1313, 1074, 925, 1207, 825, 415, 1262, 504, 529, 584, 615, 1030, 443, 982, 1090, 844, 1125, 776, 826, 966, 719, 1400, 746, 1374, 1176, 1430, 546, 1218, 1008, 473, 114, 1060, 472, 681],
    _phantom: core::marker::PhantomData,
};

pub(super) static DATAS: &[KeysymData] = &[
    KeysymData {
        keysym_or_definitive_idx: 0x00000000,
        name_start: 0,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00ffffff,
        name_start: 8,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff08,
        name_start: 18,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff09,
        name_start: 27,
        name_len: 3,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0a,
        name_start: 30,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0b,
        name_start: 38,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff0d,
        name_start: 43,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff13,
        name_start: 49,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff14,
        name_start: 54,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff15,
        name_start: 65,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff1b,
        name_start: 72,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffff,
        name_start: 78,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff20,
        name_start: 84,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000000c,
        name_start: 93,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff37,
        name_start: 103,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000000e,
        name_start: 112,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000000e,
        name_start: 124,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3c,
        name_start: 140,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000011,
        name_start: 155,
        name_len: 22,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3d,
        name_start: 177,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000013,
        name_start: 194,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000013,
        name_start: 202,
        name_len: 24,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3e,
        name_start: 226,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000016,
        name_start: 243,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000016,
        name_start: 251,
        name_len: 24,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff21,
        name_start: 275,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff22,
        name_start: 280,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff23,
        name_start: 288,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000001b,
        name_start: 299,
        name_len: 6,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff24,
        name_start: 305,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff25,
        name_start: 311,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff26,
        name_start: 319,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff27,
        name_start: 327,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff28,
        name_start: 344,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff29,
        name_start: 351,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2a,
        name_start: 358,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2b,
        name_start: 373,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2c,
        name_start: 380,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2d,
        name_start: 386,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2e,
        name_start: 395,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff2f,
        name_start: 405,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff30,
        name_start: 415,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff50,
        name_start: 426,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff51,
        name_start: 430,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff52,
        name_start: 434,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff53,
        name_start: 436,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff54,
        name_start: 441,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff55,
        name_start: 445,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002f,
        name_start: 450,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002f,
        name_start: 457,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff56,
        name_start: 466,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000032,
        name_start: 470,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000032,
        name_start: 479,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff57,
        name_start: 490,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff58,
        name_start: 493,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff60,
        name_start: 498,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff61,
        name_start: 504,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000038,
        name_start: 509,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff62,
        name_start: 524,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff63,
        name_start: 531,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff65,
        name_start: 537,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003c,
        name_start: 541,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff66,
        name_start: 548,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003e,
        name_start: 552,
        name_len: 8,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff67,
        name_start: 560,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff68,
        name_start: 564,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000041,
        name_start: 568,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff69,
        name_start: 575,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000043,
        name_start: 581,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff6a,
        name_start: 588,
        name_len: 4,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff6b,
        name_start: 592,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff7e,
        name_start: 597,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 608,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 621,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 636,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 647,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 660,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 672,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 685,
        name_len: 13,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 698,
        name_len: 11,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff7f,
        name_start: 709,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff80,
        name_start: 717,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff89,
        name_start: 725,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff8d,
        name_start: 731,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff91,
        name_start: 739,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff92,
        name_start: 744,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff93,
        name_start: 749,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff94,
        name_start: 754,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff95,
        name_start: 759,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff96,
        name_start: 766,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff97,
        name_start: 773,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff98,
        name_start: 778,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff99,
        name_start: 786,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9a,
        name_start: 793,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005d,
        name_start: 801,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9b,
        name_start: 811,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005f,
        name_start: 818,
        name_len: 12,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9c,
        name_start: 830,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9d,
        name_start: 836,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9e,
        name_start: 844,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff9f,
        name_start: 853,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbd,
        name_start: 862,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffaa,
        name_start: 870,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffab,
        name_start: 881,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffac,
        name_start: 887,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffad,
        name_start: 899,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffae,
        name_start: 910,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffaf,
        name_start: 920,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb0,
        name_start: 929,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb1,
        name_start: 933,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb2,
        name_start: 937,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb3,
        name_start: 941,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb4,
        name_start: 945,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb5,
        name_start: 949,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb6,
        name_start: 953,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb7,
        name_start: 957,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb8,
        name_start: 961,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffb9,
        name_start: 965,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbe,
        name_start: 969,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffbf,
        name_start: 971,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc0,
        name_start: 973,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc1,
        name_start: 975,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc2,
        name_start: 977,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc3,
        name_start: 979,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc4,
        name_start: 981,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc5,
        name_start: 983,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc6,
        name_start: 985,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc7,
        name_start: 987,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc8,
        name_start: 990,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000080,
        name_start: 993,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffc9,
        name_start: 995,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000082,
        name_start: 998,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffca,
        name_start: 1000,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000084,
        name_start: 1003,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcb,
        name_start: 1005,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000086,
        name_start: 1008,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcc,
        name_start: 1010,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000088,
        name_start: 1013,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcd,
        name_start: 1015,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000008a,
        name_start: 1018,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffce,
        name_start: 1020,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000008c,
        name_start: 1023,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffcf,
        name_start: 1025,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000008e,
        name_start: 1028,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd0,
        name_start: 1030,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000090,
        name_start: 1033,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd1,
        name_start: 1035,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000092,
        name_start: 1038,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd2,
        name_start: 1041,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000094,
        name_start: 1044,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd3,
        name_start: 1046,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000096,
        name_start: 1049,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd4,
        name_start: 1051,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000098,
        name_start: 1054,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd5,
        name_start: 1056,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000009a,
        name_start: 1059,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd6,
        name_start: 1061,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000009c,
        name_start: 1064,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd7,
        name_start: 1066,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000009e,
        name_start: 1069,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd8,
        name_start: 1071,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a0,
        name_start: 1074,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffd9,
        name_start: 1076,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a2,
        name_start: 1079,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffda,
        name_start: 1081,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a4,
        name_start: 1084,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdb,
        name_start: 1086,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a6,
        name_start: 1089,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdc,
        name_start: 1092,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a8,
        name_start: 1095,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdd,
        name_start: 1098,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000aa,
        name_start: 1101,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffde,
        name_start: 1104,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ac,
        name_start: 1107,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffdf,
        name_start: 1110,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ae,
        name_start: 1113,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe0,
        name_start: 1116,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b0,
        name_start: 1119,
        name_len: 3,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe1,
        name_start: 1122,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe2,
        name_start: 1129,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe3,
        name_start: 1136,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe4,
        name_start: 1145,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe5,
        name_start: 1154,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe6,
        name_start: 1163,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe7,
        name_start: 1173,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe8,
        name_start: 1179,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffe9,
        name_start: 1185,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffea,
        name_start: 1190,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffeb,
        name_start: 1195,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffec,
        name_start: 1202,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffed,
        name_start: 1209,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ffee,
        name_start: 1216,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe01,
        name_start: 1223,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe02,
        name_start: 1231,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe03,
        name_start: 1247,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe04,
        name_start: 1263,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe05,
        name_start: 1279,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe11,
        name_start: 1294,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe12,
        name_start: 1310,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe13,
        name_start: 1326,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe06,
        name_start: 1341,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe07,
        name_start: 1356,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe08,
        name_start: 1370,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe09,
        name_start: 1384,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0a,
        name_start: 1403,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0b,
        name_start: 1417,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0c,
        name_start: 1436,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0d,
        name_start: 1451,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0e,
        name_start: 1471,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe0f,
        name_start: 1485,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe20,
        name_start: 1504,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe21,
        name_start: 1516,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe22,
        name_start: 1532,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe23,
        name_start: 1550,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe24,
        name_start: 1569,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe25,
        name_start: 1590,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe26,
        name_start: 1612,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe27,
        name_start: 1635,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe28,
        name_start: 1654,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe29,
        name_start: 1674,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2a,
        name_start: 1697,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2b,
        name_start: 1721,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2c,
        name_start: 1745,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2d,
        name_start: 1765,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2e,
        name_start: 1786,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe2f,
        name_start: 1804,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe30,
        name_start: 1824,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe31,
        name_start: 1848,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe32,
        name_start: 1875,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe33,
        name_start: 1888,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe34,
        name_start: 1905,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe50,
        name_start: 1914,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe51,
        name_start: 1924,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe52,
        name_start: 1934,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe53,
        name_start: 1949,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ea,
        name_start: 1959,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe54,
        name_start: 1975,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe55,
        name_start: 1986,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe56,
        name_start: 1996,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe57,
        name_start: 2009,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe58,
        name_start: 2023,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe59,
        name_start: 2037,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5a,
        name_start: 2053,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5b,
        name_start: 2063,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5c,
        name_start: 2075,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5d,
        name_start: 2086,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5e,
        name_start: 2095,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe5f,
        name_start: 2112,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe60,
        name_start: 2133,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe61,
        name_start: 2146,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe62,
        name_start: 2155,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe63,
        name_start: 2164,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe64,
        name_start: 2175,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fc,
        name_start: 2190,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe65,
        name_start: 2200,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fe,
        name_start: 2223,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe66,
        name_start: 2233,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe67,
        name_start: 2249,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe68,
        name_start: 2263,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe69,
        name_start: 2279,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6a,
        name_start: 2299,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6b,
        name_start: 2314,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6c,
        name_start: 2329,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6d,
        name_start: 2348,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6e,
        name_start: 2366,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe6f,
        name_start: 2381,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe90,
        name_start: 2394,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe91,
        name_start: 2406,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe92,
        name_start: 2428,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe93,
        name_start: 2450,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe80,
        name_start: 2473,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe81,
        name_start: 2479,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe82,
        name_start: 2485,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe83,
        name_start: 2491,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe84,
        name_start: 2497,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe85,
        name_start: 2503,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe86,
        name_start: 2509,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe87,
        name_start: 2515,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe88,
        name_start: 2521,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe89,
        name_start: 2527,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8a,
        name_start: 2533,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000118,
        name_start: 2549,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8b,
        name_start: 2559,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000011a,
        name_start: 2577,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8c,
        name_start: 2587,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe8d,
        name_start: 2597,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed0,
        name_start: 2607,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed1,
        name_start: 2627,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed2,
        name_start: 2646,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed4,
        name_start: 2665,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fed5,
        name_start: 2684,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe70,
        name_start: 2700,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe71,
        name_start: 2714,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe72,
        name_start: 2737,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe73,
        name_start: 2754,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe74,
        name_start: 2769,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe75,
        name_start: 2786,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe76,
        name_start: 2803,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe77,
        name_start: 2819,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe78,
        name_start: 2841,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe79,
        name_start: 2856,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fe7a,
        name_start: 2871,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee0,
        name_start: 2889,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee1,
        name_start: 2901,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee2,
        name_start: 2914,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee3,
        name_start: 2924,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee4,
        name_start: 2936,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee5,
        name_start: 2950,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee6,
        name_start: 2965,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee7,
        name_start: 2981,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee8,
        name_start: 2998,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fee9,
        name_start: 3017,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feea,
        name_start: 3032,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feeb,
        name_start: 3047,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feec,
        name_start: 3062,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feed,
        name_start: 3077,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feee,
        name_start: 3092,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000feef,
        name_start: 3113,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef0,
        name_start: 3130,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef1,
        name_start: 3147,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef2,
        name_start: 3164,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef3,
        name_start: 3181,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef4,
        name_start: 3198,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef5,
        name_start: 3215,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef6,
        name_start: 3228,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef7,
        name_start: 3241,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef8,
        name_start: 3254,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefd,
        name_start: 3267,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fef9,
        name_start: 3280,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefa,
        name_start: 3298,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefb,
        name_start: 3316,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fefc,
        name_start: 3335,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea0,
        name_start: 3354,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea1,
        name_start: 3356,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea2,
        name_start: 3358,
        name_len: 2,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea3,
        name_start: 3360,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea4,
        name_start: 3363,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fea5,
        name_start: 3366,
        name_len: 3,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd01,
        name_start: 3369,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd02,
        name_start: 3383,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd03,
        name_start: 3397,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd04,
        name_start: 3408,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd05,
        name_start: 3418,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd06,
        name_start: 3430,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd07,
        name_start: 3443,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd08,
        name_start: 3458,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd09,
        name_start: 3468,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0a,
        name_start: 3477,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0b,
        name_start: 3485,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0c,
        name_start: 3493,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0d,
        name_start: 3501,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0e,
        name_start: 3510,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd0f,
        name_start: 3519,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd10,
        name_start: 3535,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd11,
        name_start: 3549,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd12,
        name_start: 3562,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd13,
        name_start: 3571,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd14,
        name_start: 3581,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd15,
        name_start: 3590,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd16,
        name_start: 3599,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd17,
        name_start: 3608,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd18,
        name_start: 3618,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd19,
        name_start: 3629,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1a,
        name_start: 3646,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1b,
        name_start: 3661,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1c,
        name_start: 3674,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1d,
        name_start: 3691,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fd1e,
        name_start: 3707,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000020,
        name_start: 3717,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000021,
        name_start: 3722,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000022,
        name_start: 3728,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000023,
        name_start: 3736,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000024,
        name_start: 3746,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000025,
        name_start: 3752,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000026,
        name_start: 3759,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000027,
        name_start: 3768,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000177,
        name_start: 3778,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000028,
        name_start: 3788,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000029,
        name_start: 3797,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002a,
        name_start: 3807,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002b,
        name_start: 3815,
        name_len: 4,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002c,
        name_start: 3819,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002d,
        name_start: 3824,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002e,
        name_start: 3829,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000002f,
        name_start: 3835,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000030,
        name_start: 3840,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000031,
        name_start: 3841,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000032,
        name_start: 3842,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000033,
        name_start: 3843,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000034,
        name_start: 3844,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000035,
        name_start: 3845,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000036,
        name_start: 3846,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000037,
        name_start: 3847,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000038,
        name_start: 3848,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000039,
        name_start: 3849,
        name_len: 1,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003a,
        name_start: 3850,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003b,
        name_start: 3855,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003c,
        name_start: 3864,
        name_len: 4,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003d,
        name_start: 3868,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003e,
        name_start: 3873,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000003f,
        name_start: 3880,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000040,
        name_start: 3888,
        name_len: 2,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000041,
        name_start: 3890,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000042,
        name_start: 3891,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000043,
        name_start: 3892,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000044,
        name_start: 3893,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000045,
        name_start: 3894,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000046,
        name_start: 3895,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000047,
        name_start: 3896,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000048,
        name_start: 3897,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000049,
        name_start: 3898,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004a,
        name_start: 3899,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004b,
        name_start: 3900,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004c,
        name_start: 3901,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004d,
        name_start: 3902,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004e,
        name_start: 3903,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000004f,
        name_start: 3904,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000050,
        name_start: 3905,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000051,
        name_start: 3906,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000052,
        name_start: 3907,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000053,
        name_start: 3908,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000054,
        name_start: 3909,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000055,
        name_start: 3910,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000056,
        name_start: 3911,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000057,
        name_start: 3912,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000058,
        name_start: 3913,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000059,
        name_start: 3914,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005a,
        name_start: 3915,
        name_len: 1,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005b,
        name_start: 3916,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005c,
        name_start: 3927,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005d,
        name_start: 3936,
        name_len: 12,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005e,
        name_start: 3948,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000005f,
        name_start: 3959,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000060,
        name_start: 3969,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b1,
        name_start: 3974,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000061,
        name_start: 3983,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000062,
        name_start: 3984,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000063,
        name_start: 3985,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000064,
        name_start: 3986,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000065,
        name_start: 3987,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000066,
        name_start: 3988,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000067,
        name_start: 3989,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000068,
        name_start: 3990,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000069,
        name_start: 3991,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006a,
        name_start: 3992,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006b,
        name_start: 3993,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006c,
        name_start: 3994,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006d,
        name_start: 3995,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006e,
        name_start: 3996,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000006f,
        name_start: 3997,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000070,
        name_start: 3998,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000071,
        name_start: 3999,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000072,
        name_start: 4000,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000073,
        name_start: 4001,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000074,
        name_start: 4002,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000075,
        name_start: 4003,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000076,
        name_start: 4004,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000077,
        name_start: 4005,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000078,
        name_start: 4006,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000079,
        name_start: 4007,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007a,
        name_start: 4008,
        name_len: 1,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007b,
        name_start: 4009,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007c,
        name_start: 4018,
        name_len: 3,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007d,
        name_start: 4021,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000007e,
        name_start: 4031,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a0,
        name_start: 4041,
        name_len: 12,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a1,
        name_start: 4053,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a2,
        name_start: 4063,
        name_len: 4,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a3,
        name_start: 4067,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a4,
        name_start: 4075,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a5,
        name_start: 4083,
        name_len: 3,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a6,
        name_start: 4086,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a7,
        name_start: 4095,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a8,
        name_start: 4102,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000a9,
        name_start: 4111,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000aa,
        name_start: 4120,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ab,
        name_start: 4131,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001dc,
        name_start: 4144,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ac,
        name_start: 4157,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ad,
        name_start: 4164,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ae,
        name_start: 4170,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000af,
        name_start: 4180,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b0,
        name_start: 4186,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b1,
        name_start: 4192,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b2,
        name_start: 4201,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b3,
        name_start: 4212,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b4,
        name_start: 4225,
        name_len: 5,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b5,
        name_start: 4230,
        name_len: 2,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b6,
        name_start: 4232,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b7,
        name_start: 4241,
        name_len: 14,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b8,
        name_start: 4255,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000b9,
        name_start: 4262,
        name_len: 11,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ba,
        name_start: 4273,
        name_len: 9,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ec,
        name_start: 4282,
        name_len: 12,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bb,
        name_start: 4294,
        name_len: 14,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ee,
        name_start: 4308,
        name_len: 14,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bc,
        name_start: 4322,
        name_len: 10,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bd,
        name_start: 4332,
        name_len: 7,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000be,
        name_start: 4339,
        name_len: 13,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000bf,
        name_start: 4352,
        name_len: 12,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c0,
        name_start: 4364,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c1,
        name_start: 4370,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c2,
        name_start: 4376,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c3,
        name_start: 4387,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c4,
        name_start: 4393,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c5,
        name_start: 4403,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c6,
        name_start: 4408,
        name_len: 2,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c7,
        name_start: 4410,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c8,
        name_start: 4418,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000c9,
        name_start: 4424,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ca,
        name_start: 4430,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cb,
        name_start: 4441,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cc,
        name_start: 4451,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cd,
        name_start: 4457,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ce,
        name_start: 4463,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000cf,
        name_start: 4474,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d0,
        name_start: 4484,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000204,
        name_start: 4487,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d1,
        name_start: 4490,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d2,
        name_start: 4496,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d3,
        name_start: 4502,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d4,
        name_start: 4508,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d5,
        name_start: 4519,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d6,
        name_start: 4525,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d7,
        name_start: 4535,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d8,
        name_start: 4543,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000020d,
        name_start: 4549,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000d9,
        name_start: 4557,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000da,
        name_start: 4563,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000db,
        name_start: 4569,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000dc,
        name_start: 4580,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000dd,
        name_start: 4590,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000de,
        name_start: 4596,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000214,
        name_start: 4601,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000df,
        name_start: 4606,
        name_len: 6,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e0,
        name_start: 4612,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e1,
        name_start: 4618,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e2,
        name_start: 4624,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e3,
        name_start: 4635,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e4,
        name_start: 4641,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e5,
        name_start: 4651,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e6,
        name_start: 4656,
        name_len: 2,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e7,
        name_start: 4658,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e8,
        name_start: 4666,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000e9,
        name_start: 4672,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ea,
        name_start: 4678,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000eb,
        name_start: 4689,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ec,
        name_start: 4699,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ed,
        name_start: 4705,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ee,
        name_start: 4711,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ef,
        name_start: 4722,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f0,
        name_start: 4732,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f1,
        name_start: 4735,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f2,
        name_start: 4741,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f3,
        name_start: 4747,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f4,
        name_start: 4753,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f5,
        name_start: 4764,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f6,
        name_start: 4770,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f7,
        name_start: 4780,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f8,
        name_start: 4788,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000022f,
        name_start: 4794,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000f9,
        name_start: 4802,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fa,
        name_start: 4808,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fb,
        name_start: 4814,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fc,
        name_start: 4825,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fd,
        name_start: 4835,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000fe,
        name_start: 4841,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000000ff,
        name_start: 4846,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a1,
        name_start: 4856,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a2,
        name_start: 4863,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a3,
        name_start: 4868,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a5,
        name_start: 4875,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a6,
        name_start: 4881,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001a9,
        name_start: 4887,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001aa,
        name_start: 4893,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ab,
        name_start: 4901,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ac,
        name_start: 4907,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ae,
        name_start: 4913,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001af,
        name_start: 4919,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b1,
        name_start: 4928,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b2,
        name_start: 4935,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b3,
        name_start: 4941,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b5,
        name_start: 4948,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b6,
        name_start: 4954,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b7,
        name_start: 4960,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001b9,
        name_start: 4965,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ba,
        name_start: 4971,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bb,
        name_start: 4979,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bc,
        name_start: 4985,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bd,
        name_start: 4991,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001be,
        name_start: 5002,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001bf,
        name_start: 5008,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c0,
        name_start: 5017,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c3,
        name_start: 5023,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c5,
        name_start: 5029,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c6,
        name_start: 5035,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001c8,
        name_start: 5041,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ca,
        name_start: 5047,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001cc,
        name_start: 5054,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001cf,
        name_start: 5060,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d0,
        name_start: 5066,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d1,
        name_start: 5073,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d2,
        name_start: 5079,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d5,
        name_start: 5085,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d8,
        name_start: 5097,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001d9,
        name_start: 5103,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001db,
        name_start: 5108,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001de,
        name_start: 5120,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e0,
        name_start: 5128,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e3,
        name_start: 5134,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e5,
        name_start: 5140,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e6,
        name_start: 5146,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001e8,
        name_start: 5152,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ea,
        name_start: 5158,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ec,
        name_start: 5165,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ef,
        name_start: 5171,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f0,
        name_start: 5177,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f1,
        name_start: 5184,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f2,
        name_start: 5190,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f5,
        name_start: 5196,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f8,
        name_start: 5208,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001f9,
        name_start: 5214,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001fb,
        name_start: 5219,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001fe,
        name_start: 5231,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000001ff,
        name_start: 5239,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002a1,
        name_start: 5247,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002a6,
        name_start: 5254,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002a9,
        name_start: 5265,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002ab,
        name_start: 5274,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002ac,
        name_start: 5280,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002b1,
        name_start: 5291,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002b6,
        name_start: 5298,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002b9,
        name_start: 5309,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002bb,
        name_start: 5317,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002bc,
        name_start: 5323,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002c5,
        name_start: 5334,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002c6,
        name_start: 5343,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002d5,
        name_start: 5354,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002d8,
        name_start: 5363,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002dd,
        name_start: 5374,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002de,
        name_start: 5380,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002e5,
        name_start: 5391,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002e6,
        name_start: 5400,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002f5,
        name_start: 5411,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002f8,
        name_start: 5420,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002fd,
        name_start: 5431,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002fe,
        name_start: 5437,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a2,
        name_start: 5448,
        name_len: 3,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000287,
        name_start: 5451,
        name_len: 5,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a3,
        name_start: 5456,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a5,
        name_start: 5464,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a6,
        name_start: 5470,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003aa,
        name_start: 5478,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ab,
        name_start: 5485,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ac,
        name_start: 5493,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003b3,
        name_start: 5499,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003b5,
        name_start: 5507,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003b6,
        name_start: 5513,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ba,
        name_start: 5521,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bb,
        name_start: 5528,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bc,
        name_start: 5536,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bd,
        name_start: 5542,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003bf,
        name_start: 5545,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003c0,
        name_start: 5548,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003c7,
        name_start: 5555,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003cc,
        name_start: 5562,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003cf,
        name_start: 5571,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d1,
        name_start: 5578,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d2,
        name_start: 5586,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d3,
        name_start: 5593,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003d9,
        name_start: 5601,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003dd,
        name_start: 5608,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003de,
        name_start: 5614,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003e0,
        name_start: 5621,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003e7,
        name_start: 5628,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ec,
        name_start: 5635,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003ef,
        name_start: 5644,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f1,
        name_start: 5651,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f2,
        name_start: 5659,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f3,
        name_start: 5666,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f9,
        name_start: 5674,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003fd,
        name_start: 5681,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003fe,
        name_start: 5687,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000174,
        name_start: 5694,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000175,
        name_start: 5705,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000176,
        name_start: 5716,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000177,
        name_start: 5727,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e02,
        name_start: 5738,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e03,
        name_start: 5747,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e0a,
        name_start: 5756,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e0b,
        name_start: 5765,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e1e,
        name_start: 5774,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e1f,
        name_start: 5783,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e40,
        name_start: 5792,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e41,
        name_start: 5801,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e56,
        name_start: 5810,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e57,
        name_start: 5819,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e60,
        name_start: 5828,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e61,
        name_start: 5837,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e6a,
        name_start: 5846,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e6b,
        name_start: 5855,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e80,
        name_start: 5864,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e81,
        name_start: 5870,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e82,
        name_start: 5876,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e83,
        name_start: 5882,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e84,
        name_start: 5888,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e85,
        name_start: 5898,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef2,
        name_start: 5908,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef3,
        name_start: 5914,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000013bc,
        name_start: 5920,
        name_len: 2,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000013bd,
        name_start: 5922,
        name_len: 2,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000013be,
        name_start: 5924,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000047e,
        name_start: 5934,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a1,
        name_start: 5942,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a2,
        name_start: 5955,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a3,
        name_start: 5974,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a4,
        name_start: 5993,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a5,
        name_start: 6003,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002cd,
        name_start: 6019,
        name_len: 14,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a6,
        name_start: 6033,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a7,
        name_start: 6040,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a8,
        name_start: 6046,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004a9,
        name_start: 6052,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004aa,
        name_start: 6058,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ab,
        name_start: 6064,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ac,
        name_start: 6070,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ad,
        name_start: 6077,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ae,
        name_start: 6084,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004af,
        name_start: 6091,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002d8,
        name_start: 6099,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b0,
        name_start: 6106,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b1,
        name_start: 6120,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b2,
        name_start: 6126,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b3,
        name_start: 6132,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b4,
        name_start: 6138,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b5,
        name_start: 6144,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b6,
        name_start: 6150,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b7,
        name_start: 6157,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b8,
        name_start: 6164,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004b9,
        name_start: 6171,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ba,
        name_start: 6178,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bb,
        name_start: 6185,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bc,
        name_start: 6192,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bd,
        name_start: 6200,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004be,
        name_start: 6207,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004bf,
        name_start: 6214,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c0,
        name_start: 6221,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c1,
        name_start: 6228,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002eb,
        name_start: 6236,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c2,
        name_start: 6243,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002ed,
        name_start: 6251,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c3,
        name_start: 6258,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c4,
        name_start: 6265,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c5,
        name_start: 6272,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c6,
        name_start: 6279,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c7,
        name_start: 6286,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c8,
        name_start: 6293,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004c9,
        name_start: 6300,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ca,
        name_start: 6307,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cb,
        name_start: 6314,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cc,
        name_start: 6321,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000002f8,
        name_start: 6328,
        name_len: 7,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cd,
        name_start: 6335,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ce,
        name_start: 6342,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004cf,
        name_start: 6349,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d0,
        name_start: 6356,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d1,
        name_start: 6363,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d2,
        name_start: 6370,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d3,
        name_start: 6377,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d4,
        name_start: 6384,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d5,
        name_start: 6391,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d6,
        name_start: 6398,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d7,
        name_start: 6405,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d8,
        name_start: 6412,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004d9,
        name_start: 6419,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004da,
        name_start: 6426,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004db,
        name_start: 6433,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004dc,
        name_start: 6440,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004dd,
        name_start: 6447,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004de,
        name_start: 6453,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004df,
        name_start: 6464,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f0,
        name_start: 6479,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f1,
        name_start: 6486,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f2,
        name_start: 6493,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f3,
        name_start: 6500,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f4,
        name_start: 6507,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f5,
        name_start: 6514,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f6,
        name_start: 6521,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f7,
        name_start: 6528,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f8,
        name_start: 6535,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006f9,
        name_start: 6542,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100066a,
        name_start: 6549,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000670,
        name_start: 6563,
        name_len: 23,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000679,
        name_start: 6586,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100067e,
        name_start: 6597,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000686,
        name_start: 6607,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000688,
        name_start: 6619,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000691,
        name_start: 6630,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ac,
        name_start: 6641,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006d4,
        name_start: 6653,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000660,
        name_start: 6668,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000661,
        name_start: 6676,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000662,
        name_start: 6684,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000663,
        name_start: 6692,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000664,
        name_start: 6700,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000665,
        name_start: 6708,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000666,
        name_start: 6716,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000667,
        name_start: 6724,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000668,
        name_start: 6732,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000669,
        name_start: 6740,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005bb,
        name_start: 6748,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005bf,
        name_start: 6764,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c1,
        name_start: 6784,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c2,
        name_start: 6796,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c3,
        name_start: 6814,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c4,
        name_start: 6832,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c5,
        name_start: 6849,
        name_len: 21,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c6,
        name_start: 6870,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c7,
        name_start: 6887,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c8,
        name_start: 6898,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005c9,
        name_start: 6908,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ca,
        name_start: 6925,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cb,
        name_start: 6935,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cc,
        name_start: 6946,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cd,
        name_start: 6957,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ce,
        name_start: 6967,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005cf,
        name_start: 6978,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d0,
        name_start: 6988,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d1,
        name_start: 6999,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d2,
        name_start: 7008,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d3,
        name_start: 7019,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d4,
        name_start: 7030,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d5,
        name_start: 7042,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d6,
        name_start: 7052,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d7,
        name_start: 7062,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d8,
        name_start: 7072,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d9,
        name_start: 7082,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005da,
        name_start: 7092,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e0,
        name_start: 7104,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e1,
        name_start: 7118,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e2,
        name_start: 7128,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e3,
        name_start: 7138,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e4,
        name_start: 7148,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e5,
        name_start: 7158,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e6,
        name_start: 7169,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e7,
        name_start: 7180,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000034d,
        name_start: 7189,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e8,
        name_start: 7199,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005e9,
        name_start: 7209,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ea,
        name_start: 7227,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005eb,
        name_start: 7237,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ec,
        name_start: 7252,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ed,
        name_start: 7267,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ee,
        name_start: 7282,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ef,
        name_start: 7294,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005f0,
        name_start: 7306,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005f1,
        name_start: 7318,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005f2,
        name_start: 7331,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000653,
        name_start: 7343,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000654,
        name_start: 7361,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000655,
        name_start: 7379,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000698,
        name_start: 7397,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006a4,
        name_start: 7407,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006a9,
        name_start: 7417,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006af,
        name_start: 7429,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006ba,
        name_start: 7439,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006be,
        name_start: 7457,
        name_len: 22,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006cc,
        name_start: 7479,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000363,
        name_start: 7488,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006d2,
        name_start: 7504,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010006c1,
        name_start: 7520,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000492,
        name_start: 7535,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000493,
        name_start: 7551,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000496,
        name_start: 7567,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000497,
        name_start: 7589,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049a,
        name_start: 7611,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049b,
        name_start: 7632,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049c,
        name_start: 7653,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100049d,
        name_start: 7675,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004a2,
        name_start: 7697,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004a3,
        name_start: 7718,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ae,
        name_start: 7739,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004af,
        name_start: 7758,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b0,
        name_start: 7777,
        name_len: 23,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b1,
        name_start: 7800,
        name_len: 23,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b2,
        name_start: 7823,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b3,
        name_start: 7844,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b6,
        name_start: 7865,
        name_len: 22,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b7,
        name_start: 7887,
        name_len: 22,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b8,
        name_start: 7909,
        name_len: 23,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004b9,
        name_start: 7932,
        name_len: 23,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ba,
        name_start: 7955,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004bb,
        name_start: 7968,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004d8,
        name_start: 7981,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004d9,
        name_start: 7995,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e2,
        name_start: 8009,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e3,
        name_start: 8026,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e8,
        name_start: 8043,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004e9,
        name_start: 8057,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ee,
        name_start: 8071,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010004ef,
        name_start: 8088,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a1,
        name_start: 8105,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a2,
        name_start: 8116,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a3,
        name_start: 8129,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a4,
        name_start: 8140,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000388,
        name_start: 8152,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a5,
        name_start: 8163,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a6,
        name_start: 8176,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000038b,
        name_start: 8187,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a7,
        name_start: 8197,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000038d,
        name_start: 8209,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a8,
        name_start: 8220,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000038f,
        name_start: 8231,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006a9,
        name_start: 8241,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000391,
        name_start: 8253,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006aa,
        name_start: 8264,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000393,
        name_start: 8276,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ab,
        name_start: 8287,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ac,
        name_start: 8299,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ad,
        name_start: 8312,
        name_len: 25,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ae,
        name_start: 8337,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006af,
        name_start: 8356,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000399,
        name_start: 8369,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b0,
        name_start: 8380,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b1,
        name_start: 8390,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b2,
        name_start: 8401,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b3,
        name_start: 8414,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b4,
        name_start: 8425,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000039f,
        name_start: 8437,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b5,
        name_start: 8448,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b6,
        name_start: 8461,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a2,
        name_start: 8472,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b7,
        name_start: 8482,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a4,
        name_start: 8494,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b8,
        name_start: 8505,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a6,
        name_start: 8516,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006b9,
        name_start: 8526,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003a8,
        name_start: 8538,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ba,
        name_start: 8549,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003aa,
        name_start: 8561,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bb,
        name_start: 8572,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bc,
        name_start: 8584,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bd,
        name_start: 8597,
        name_len: 25,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006be,
        name_start: 8622,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006bf,
        name_start: 8641,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003b0,
        name_start: 8654,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c0,
        name_start: 8665,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c1,
        name_start: 8676,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c2,
        name_start: 8686,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c3,
        name_start: 8697,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c4,
        name_start: 8709,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c5,
        name_start: 8720,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c6,
        name_start: 8731,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c7,
        name_start: 8742,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c8,
        name_start: 8754,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006c9,
        name_start: 8765,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ca,
        name_start: 8775,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cb,
        name_start: 8790,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cc,
        name_start: 8801,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cd,
        name_start: 8812,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ce,
        name_start: 8823,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006cf,
        name_start: 8834,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d0,
        name_start: 8844,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d1,
        name_start: 8855,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d2,
        name_start: 8866,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d3,
        name_start: 8877,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d4,
        name_start: 8888,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d5,
        name_start: 8899,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d6,
        name_start: 8909,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d7,
        name_start: 8921,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d8,
        name_start: 8932,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006d9,
        name_start: 8949,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006da,
        name_start: 8962,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006db,
        name_start: 8973,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006dc,
        name_start: 8985,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006dd,
        name_start: 8995,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006de,
        name_start: 9009,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006df,
        name_start: 9021,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e0,
        name_start: 9038,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e1,
        name_start: 9049,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e2,
        name_start: 9059,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e3,
        name_start: 9070,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e4,
        name_start: 9082,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e5,
        name_start: 9093,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e6,
        name_start: 9104,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e7,
        name_start: 9115,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e8,
        name_start: 9127,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006e9,
        name_start: 9138,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ea,
        name_start: 9148,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006eb,
        name_start: 9163,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ec,
        name_start: 9174,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ed,
        name_start: 9185,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ee,
        name_start: 9196,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ef,
        name_start: 9207,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f0,
        name_start: 9217,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f1,
        name_start: 9228,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f2,
        name_start: 9239,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f3,
        name_start: 9250,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f4,
        name_start: 9261,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f5,
        name_start: 9272,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f6,
        name_start: 9282,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f7,
        name_start: 9294,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f8,
        name_start: 9305,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006f9,
        name_start: 9322,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fa,
        name_start: 9335,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fb,
        name_start: 9346,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fc,
        name_start: 9358,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fd,
        name_start: 9368,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006fe,
        name_start: 9382,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000006ff,
        name_start: 9394,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a1,
        name_start: 9411,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a2,
        name_start: 9428,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a3,
        name_start: 9447,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a4,
        name_start: 9462,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a5,
        name_start: 9478,
        name_len: 18,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000003f6,
        name_start: 9496,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a7,
        name_start: 9515,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a8,
        name_start: 9534,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007a9,
        name_start: 9553,
        name_len: 21,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ab,
        name_start: 9574,
        name_len: 17,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ae,
        name_start: 9591,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007af,
        name_start: 9611,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b1,
        name_start: 9625,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b2,
        name_start: 9642,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b3,
        name_start: 9661,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b4,
        name_start: 9676,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b5,
        name_start: 9692,
        name_len: 18,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b6,
        name_start: 9710,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b7,
        name_start: 9734,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b8,
        name_start: 9753,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007b9,
        name_start: 9772,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ba,
        name_start: 9793,
        name_len: 27,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007bb,
        name_start: 9820,
        name_len: 17,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c1,
        name_start: 9837,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c2,
        name_start: 9848,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c3,
        name_start: 9858,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c4,
        name_start: 9869,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c5,
        name_start: 9880,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c6,
        name_start: 9893,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c7,
        name_start: 9903,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c8,
        name_start: 9912,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007c9,
        name_start: 9923,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ca,
        name_start: 9933,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cb,
        name_start: 9944,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000413,
        name_start: 9955,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cc,
        name_start: 9967,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cd,
        name_start: 9975,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ce,
        name_start: 9983,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007cf,
        name_start: 9991,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d0,
        name_start: 10004,
        name_len: 8,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d1,
        name_start: 10012,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d2,
        name_start: 10021,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d4,
        name_start: 10032,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d5,
        name_start: 10041,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d6,
        name_start: 10054,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d7,
        name_start: 10063,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d8,
        name_start: 10072,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007d9,
        name_start: 10081,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e1,
        name_start: 10092,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e2,
        name_start: 10103,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e3,
        name_start: 10113,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e4,
        name_start: 10124,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e5,
        name_start: 10135,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e6,
        name_start: 10148,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e7,
        name_start: 10158,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e8,
        name_start: 10167,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007e9,
        name_start: 10178,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ea,
        name_start: 10188,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007eb,
        name_start: 10199,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000042c,
        name_start: 10210,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ec,
        name_start: 10222,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ed,
        name_start: 10230,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ee,
        name_start: 10238,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007ef,
        name_start: 10246,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f0,
        name_start: 10259,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f1,
        name_start: 10267,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f2,
        name_start: 10276,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f3,
        name_start: 10287,
        name_len: 21,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f4,
        name_start: 10308,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f5,
        name_start: 10317,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f6,
        name_start: 10330,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f7,
        name_start: 10339,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f8,
        name_start: 10348,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000007f9,
        name_start: 10357,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a1,
        name_start: 10368,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a2,
        name_start: 10379,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a3,
        name_start: 10393,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a4,
        name_start: 10407,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a5,
        name_start: 10418,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a6,
        name_start: 10429,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a7,
        name_start: 10442,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a8,
        name_start: 10458,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008a9,
        name_start: 10474,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008aa,
        name_start: 10491,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ab,
        name_start: 10508,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ac,
        name_start: 10521,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ad,
        name_start: 10534,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ae,
        name_start: 10548,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008af,
        name_start: 10562,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b0,
        name_start: 10582,
        name_len: 21,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b1,
        name_start: 10603,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b2,
        name_start: 10619,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b3,
        name_start: 10635,
        name_len: 25,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b4,
        name_start: 10660,
        name_len: 25,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b5,
        name_start: 10685,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b6,
        name_start: 10702,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008b7,
        name_start: 10719,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008bc,
        name_start: 10739,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008bd,
        name_start: 10752,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008be,
        name_start: 10760,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008bf,
        name_start: 10776,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c0,
        name_start: 10784,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c1,
        name_start: 10793,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c2,
        name_start: 10802,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c5,
        name_start: 10810,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c8,
        name_start: 10815,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008c9,
        name_start: 10826,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008cd,
        name_start: 10838,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ce,
        name_start: 10846,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008cf,
        name_start: 10853,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008d6,
        name_start: 10862,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008da,
        name_start: 10869,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008db,
        name_start: 10879,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008dc,
        name_start: 10887,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008dd,
        name_start: 10899,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008de,
        name_start: 10904,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008df,
        name_start: 10914,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008ef,
        name_start: 10923,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008f6,
        name_start: 10940,
        name_len: 8,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fb,
        name_start: 10948,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fc,
        name_start: 10957,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fd,
        name_start: 10964,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000008fe,
        name_start: 10974,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009df,
        name_start: 10983,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e0,
        name_start: 10988,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e1,
        name_start: 11000,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e2,
        name_start: 11012,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e3,
        name_start: 11014,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e4,
        name_start: 11016,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e5,
        name_start: 11018,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e8,
        name_start: 11020,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e9,
        name_start: 11022,
        name_len: 2,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ea,
        name_start: 11024,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009eb,
        name_start: 11038,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ec,
        name_start: 11051,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ed,
        name_start: 11063,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ee,
        name_start: 11076,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ef,
        name_start: 11089,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f0,
        name_start: 11103,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f1,
        name_start: 11117,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f2,
        name_start: 11131,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f3,
        name_start: 11145,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f4,
        name_start: 11159,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f5,
        name_start: 11164,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f6,
        name_start: 11170,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f7,
        name_start: 11174,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009f8,
        name_start: 11178,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa1,
        name_start: 11185,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa2,
        name_start: 11192,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa3,
        name_start: 11199,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa4,
        name_start: 11207,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa5,
        name_start: 11215,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa6,
        name_start: 11225,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa7,
        name_start: 11235,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa8,
        name_start: 11244,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aa9,
        name_start: 11253,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aaa,
        name_start: 11259,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aac,
        name_start: 11265,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aae,
        name_start: 11276,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aaf,
        name_start: 11284,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab0,
        name_start: 11299,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab1,
        name_start: 11307,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab2,
        name_start: 11316,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab3,
        name_start: 11324,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab4,
        name_start: 11333,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab5,
        name_start: 11344,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab6,
        name_start: 11354,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab7,
        name_start: 11362,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ab8,
        name_start: 11372,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abb,
        name_start: 11378,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abc,
        name_start: 11385,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abd,
        name_start: 11401,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abe,
        name_start: 11413,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000abf,
        name_start: 11430,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac3,
        name_start: 11436,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac4,
        name_start: 11445,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac5,
        name_start: 11457,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac6,
        name_start: 11468,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ac9,
        name_start: 11480,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aca,
        name_start: 11489,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acb,
        name_start: 11502,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acc,
        name_start: 11519,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acd,
        name_start: 11535,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ace,
        name_start: 11552,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000acf,
        name_start: 11564,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad0,
        name_start: 11579,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad1,
        name_start: 11598,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad2,
        name_start: 11618,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad3,
        name_start: 11637,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad4,
        name_start: 11657,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad5,
        name_start: 11669,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad6,
        name_start: 11677,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad7,
        name_start: 11684,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ad9,
        name_start: 11691,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ada,
        name_start: 11701,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000adb,
        name_start: 11709,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000adc,
        name_start: 11725,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000add,
        name_start: 11744,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ade,
        name_start: 11764,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000adf,
        name_start: 11778,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae0,
        name_start: 11790,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae1,
        name_start: 11806,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae2,
        name_start: 11824,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae3,
        name_start: 11838,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae4,
        name_start: 11853,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae5,
        name_start: 11870,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae6,
        name_start: 11878,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae7,
        name_start: 11896,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae8,
        name_start: 11912,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ae9,
        name_start: 11929,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aea,
        name_start: 11948,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aeb,
        name_start: 11959,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aec,
        name_start: 11971,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aed,
        name_start: 11975,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aee,
        name_start: 11982,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af0,
        name_start: 11987,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af1,
        name_start: 11999,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af2,
        name_start: 12005,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af3,
        name_start: 12017,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af4,
        name_start: 12026,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af5,
        name_start: 12037,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af6,
        name_start: 12049,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af7,
        name_start: 12060,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af8,
        name_start: 12070,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000af9,
        name_start: 12082,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afa,
        name_start: 12091,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afb,
        name_start: 12108,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afc,
        name_start: 12127,
        name_len: 5,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afd,
        name_start: 12132,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000afe,
        name_start: 12150,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000aff,
        name_start: 12168,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba3,
        name_start: 12174,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba6,
        name_start: 12183,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba8,
        name_start: 12193,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ba9,
        name_start: 12202,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc0,
        name_start: 12209,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc2,
        name_start: 12216,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc3,
        name_start: 12224,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc4,
        name_start: 12230,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bc6,
        name_start: 12239,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bca,
        name_start: 12247,
        name_len: 3,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bcc,
        name_start: 12250,
        name_len: 4,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bce,
        name_start: 12254,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bcf,
        name_start: 12260,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bd3,
        name_start: 12266,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bd6,
        name_start: 12273,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bd8,
        name_start: 12281,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bda,
        name_start: 12290,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bdc,
        name_start: 12298,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000bfc,
        name_start: 12306,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cdf,
        name_start: 12315,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce0,
        name_start: 12335,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce1,
        name_start: 12347,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004ee,
        name_start: 12357,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce2,
        name_start: 12368,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f0,
        name_start: 12380,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce3,
        name_start: 12393,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f2,
        name_start: 12405,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce4,
        name_start: 12418,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce5,
        name_start: 12427,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce6,
        name_start: 12437,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f6,
        name_start: 12448,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce7,
        name_start: 12460,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004f8,
        name_start: 12471,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce8,
        name_start: 12481,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000004fa,
        name_start: 12491,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ce9,
        name_start: 12502,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cea,
        name_start: 12512,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ceb,
        name_start: 12528,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cec,
        name_start: 12539,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ced,
        name_start: 12551,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cee,
        name_start: 12566,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cef,
        name_start: 12576,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf0,
        name_start: 12591,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf1,
        name_start: 12601,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000504,
        name_start: 12614,
        name_len: 13,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf2,
        name_start: 12627,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf3,
        name_start: 12638,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf4,
        name_start: 12652,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf5,
        name_start: 12661,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000509,
        name_start: 12677,
        name_len: 16,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf6,
        name_start: 12693,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000050b,
        name_start: 12704,
        name_len: 11,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf7,
        name_start: 12715,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000050d,
        name_start: 12726,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf8,
        name_start: 12736,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cf9,
        name_start: 12747,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000cfa,
        name_start: 12758,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000511,
        name_start: 12768,
        name_len: 10,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da1,
        name_start: 12778,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da2,
        name_start: 12788,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da3,
        name_start: 12800,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da4,
        name_start: 12813,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da5,
        name_start: 12826,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da6,
        name_start: 12838,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da7,
        name_start: 12853,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da8,
        name_start: 12864,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000da9,
        name_start: 12876,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000daa,
        name_start: 12889,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dab,
        name_start: 12902,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dac,
        name_start: 12911,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dad,
        name_start: 12923,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dae,
        name_start: 12934,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000daf,
        name_start: 12946,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db0,
        name_start: 12958,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db1,
        name_start: 12970,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db2,
        name_start: 12988,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db3,
        name_start: 13003,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db4,
        name_start: 13013,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db5,
        name_start: 13023,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db6,
        name_start: 13033,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db7,
        name_start: 13046,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db8,
        name_start: 13060,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000db9,
        name_start: 13073,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dba,
        name_start: 13082,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbb,
        name_start: 13095,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbc,
        name_start: 13105,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbd,
        name_start: 13118,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbe,
        name_start: 13127,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dbf,
        name_start: 13139,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc0,
        name_start: 13149,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc1,
        name_start: 13164,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc2,
        name_start: 13173,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc3,
        name_start: 13183,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc4,
        name_start: 13193,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc5,
        name_start: 13200,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc6,
        name_start: 13211,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc7,
        name_start: 13218,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc8,
        name_start: 13229,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dc9,
        name_start: 13240,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dca,
        name_start: 13251,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcb,
        name_start: 13261,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcc,
        name_start: 13271,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcd,
        name_start: 13283,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dce,
        name_start: 13292,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dcf,
        name_start: 13305,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd0,
        name_start: 13319,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd1,
        name_start: 13329,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd2,
        name_start: 13344,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd3,
        name_start: 13355,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd4,
        name_start: 13366,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd5,
        name_start: 13376,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd6,
        name_start: 13387,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd7,
        name_start: 13398,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd8,
        name_start: 13410,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dd9,
        name_start: 13420,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dda,
        name_start: 13431,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dde,
        name_start: 13443,
        name_len: 22,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ddf,
        name_start: 13465,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de0,
        name_start: 13474,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de1,
        name_start: 13484,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de2,
        name_start: 13495,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de3,
        name_start: 13505,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de4,
        name_start: 13523,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de5,
        name_start: 13542,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de6,
        name_start: 13558,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de7,
        name_start: 13571,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de8,
        name_start: 13585,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000de9,
        name_start: 13595,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dea,
        name_start: 13606,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000deb,
        name_start: 13617,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000dec,
        name_start: 13633,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ded,
        name_start: 13649,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df0,
        name_start: 13662,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df1,
        name_start: 13673,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df2,
        name_start: 13685,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df3,
        name_start: 13697,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df4,
        name_start: 13708,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df5,
        name_start: 13718,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df6,
        name_start: 13728,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df7,
        name_start: 13739,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df8,
        name_start: 13751,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000df9,
        name_start: 13763,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff31,
        name_start: 13774,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff32,
        name_start: 13780,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff33,
        name_start: 13792,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff34,
        name_start: 13802,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff35,
        name_start: 13814,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff36,
        name_start: 13825,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff38,
        name_start: 13838,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff39,
        name_start: 13851,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3a,
        name_start: 13863,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3b,
        name_start: 13878,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000ff3f,
        name_start: 13894,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea1,
        name_start: 13908,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea2,
        name_start: 13921,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea3,
        name_start: 13939,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea4,
        name_start: 13956,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea5,
        name_start: 13968,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea6,
        name_start: 13985,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea7,
        name_start: 14002,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea8,
        name_start: 14015,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ea9,
        name_start: 14033,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eaa,
        name_start: 14045,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eab,
        name_start: 14063,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eac,
        name_start: 14080,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ead,
        name_start: 14097,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eae,
        name_start: 14113,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eaf,
        name_start: 14130,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb0,
        name_start: 14148,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb1,
        name_start: 14165,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb2,
        name_start: 14177,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb3,
        name_start: 14189,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb4,
        name_start: 14206,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb5,
        name_start: 14222,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb6,
        name_start: 14233,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb7,
        name_start: 14249,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb8,
        name_start: 14261,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eb9,
        name_start: 14273,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eba,
        name_start: 14290,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebb,
        name_start: 14302,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebc,
        name_start: 14315,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebd,
        name_start: 14327,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebe,
        name_start: 14340,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ebf,
        name_start: 14352,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec0,
        name_start: 14360,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec1,
        name_start: 14369,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec2,
        name_start: 14378,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec3,
        name_start: 14388,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec4,
        name_start: 14397,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec5,
        name_start: 14405,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec6,
        name_start: 14415,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec7,
        name_start: 14424,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec8,
        name_start: 14432,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ec9,
        name_start: 14441,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eca,
        name_start: 14451,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecb,
        name_start: 14460,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecc,
        name_start: 14469,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecd,
        name_start: 14477,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ece,
        name_start: 14487,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ecf,
        name_start: 14496,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed0,
        name_start: 14505,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed1,
        name_start: 14514,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed2,
        name_start: 14523,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed3,
        name_start: 14532,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed4,
        name_start: 14540,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed5,
        name_start: 14555,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed6,
        name_start: 14575,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed7,
        name_start: 14594,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed8,
        name_start: 14608,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ed9,
        name_start: 14627,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eda,
        name_start: 14646,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edb,
        name_start: 14661,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edc,
        name_start: 14675,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edd,
        name_start: 14695,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ede,
        name_start: 14714,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000edf,
        name_start: 14733,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee0,
        name_start: 14751,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee1,
        name_start: 14770,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee2,
        name_start: 14790,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee3,
        name_start: 14809,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee4,
        name_start: 14823,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee5,
        name_start: 14837,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee6,
        name_start: 14855,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee7,
        name_start: 14868,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee8,
        name_start: 14886,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ee9,
        name_start: 14900,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eea,
        name_start: 14914,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eeb,
        name_start: 14928,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eec,
        name_start: 14943,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eed,
        name_start: 14957,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eee,
        name_start: 14972,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eef,
        name_start: 14986,
        name_len: 23,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef0,
        name_start: 15009,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef1,
        name_start: 15033,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef2,
        name_start: 15057,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef3,
        name_start: 15071,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef4,
        name_start: 15095,
        name_len: 25,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef5,
        name_start: 15120,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef6,
        name_start: 15138,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef7,
        name_start: 15150,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef8,
        name_start: 15163,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000ef9,
        name_start: 15179,
        name_len: 26,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000efa,
        name_start: 15205,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x00000eff,
        name_start: 15225,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000587,
        name_start: 15235,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000589,
        name_start: 15255,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005ce,
        name_start: 15273,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055d,
        name_start: 15290,
        name_len: 24,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d0,
        name_start: 15314,
        name_len: 12,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100058a,
        name_start: 15326,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d2,
        name_start: 15341,
        name_len: 17,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055c,
        name_start: 15358,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d4,
        name_start: 15373,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055b,
        name_start: 15388,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d6,
        name_start: 15403,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055e,
        name_start: 15418,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000005d8,
        name_start: 15435,
        name_len: 15,
        flags: 0 | HAS_CHAR | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000531,
        name_start: 15450,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000561,
        name_start: 15462,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000532,
        name_start: 15474,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000562,
        name_start: 15486,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000533,
        name_start: 15498,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000563,
        name_start: 15510,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000534,
        name_start: 15522,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000564,
        name_start: 15533,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000535,
        name_start: 15544,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000565,
        name_start: 15557,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000536,
        name_start: 15570,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000566,
        name_start: 15581,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000537,
        name_start: 15592,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000567,
        name_start: 15602,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000538,
        name_start: 15612,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000568,
        name_start: 15623,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000539,
        name_start: 15634,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000569,
        name_start: 15645,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053a,
        name_start: 15656,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056a,
        name_start: 15668,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053b,
        name_start: 15680,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056b,
        name_start: 15692,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053c,
        name_start: 15704,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056c,
        name_start: 15717,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053d,
        name_start: 15730,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056d,
        name_start: 15742,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053e,
        name_start: 15754,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056e,
        name_start: 15766,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100053f,
        name_start: 15778,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100056f,
        name_start: 15790,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000540,
        name_start: 15802,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000570,
        name_start: 15813,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000541,
        name_start: 15824,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000571,
        name_start: 15836,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000542,
        name_start: 15848,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000572,
        name_start: 15861,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000543,
        name_start: 15874,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000573,
        name_start: 15887,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000544,
        name_start: 15900,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000574,
        name_start: 15912,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000545,
        name_start: 15924,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000575,
        name_start: 15935,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000546,
        name_start: 15946,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000576,
        name_start: 15957,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000547,
        name_start: 15968,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000577,
        name_start: 15980,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000548,
        name_start: 15992,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000578,
        name_start: 16003,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000549,
        name_start: 16014,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000579,
        name_start: 16026,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054a,
        name_start: 16038,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057a,
        name_start: 16049,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054b,
        name_start: 16060,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057b,
        name_start: 16071,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054c,
        name_start: 16082,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057c,
        name_start: 16093,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054d,
        name_start: 16104,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057d,
        name_start: 16115,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054e,
        name_start: 16126,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057e,
        name_start: 16138,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100054f,
        name_start: 16150,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100057f,
        name_start: 16163,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000550,
        name_start: 16176,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000580,
        name_start: 16187,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000551,
        name_start: 16198,
        name_len: 12,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000581,
        name_start: 16210,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000552,
        name_start: 16222,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000582,
        name_start: 16235,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000553,
        name_start: 16248,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000583,
        name_start: 16261,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000554,
        name_start: 16274,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000584,
        name_start: 16285,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000555,
        name_start: 16296,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000585,
        name_start: 16306,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000556,
        name_start: 16316,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000586,
        name_start: 16327,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100055a,
        name_start: 16338,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d0,
        name_start: 16357,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d1,
        name_start: 16368,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d2,
        name_start: 16380,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d3,
        name_start: 16392,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d4,
        name_start: 16404,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d5,
        name_start: 16415,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d6,
        name_start: 16427,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d7,
        name_start: 16439,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d8,
        name_start: 16451,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010d9,
        name_start: 16462,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010da,
        name_start: 16474,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010db,
        name_start: 16486,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010dc,
        name_start: 16498,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010dd,
        name_start: 16510,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010de,
        name_start: 16521,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010df,
        name_start: 16533,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e0,
        name_start: 16546,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e1,
        name_start: 16558,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e2,
        name_start: 16570,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e3,
        name_start: 16582,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e4,
        name_start: 16593,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e5,
        name_start: 16606,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e6,
        name_start: 16619,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e7,
        name_start: 16632,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e8,
        name_start: 16644,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010e9,
        name_start: 16657,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ea,
        name_start: 16670,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010eb,
        name_start: 16682,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ec,
        name_start: 16694,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ed,
        name_start: 16706,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ee,
        name_start: 16719,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010ef,
        name_start: 16731,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f0,
        name_start: 16744,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f1,
        name_start: 16756,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f2,
        name_start: 16767,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f3,
        name_start: 16779,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f4,
        name_start: 16790,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f5,
        name_start: 16802,
        name_len: 12,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010010f6,
        name_start: 16814,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e8a,
        name_start: 16825,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100012c,
        name_start: 16834,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b5,
        name_start: 16840,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001e6,
        name_start: 16847,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001d1,
        name_start: 16853,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100019f,
        name_start: 16859,
        name_len: 7,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e8b,
        name_start: 16866,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100012d,
        name_start: 16875,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b6,
        name_start: 16881,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001e7,
        name_start: 16888,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001d2,
        name_start: 16894,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000275,
        name_start: 16900,
        name_len: 7,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100018f,
        name_start: 16907,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000259,
        name_start: 16912,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b7,
        name_start: 16917,
        name_len: 3,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000292,
        name_start: 16920,
        name_len: 3,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e36,
        name_start: 16923,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001e37,
        name_start: 16932,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea0,
        name_start: 16941,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea1,
        name_start: 16950,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea2,
        name_start: 16959,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea3,
        name_start: 16964,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea4,
        name_start: 16969,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea5,
        name_start: 16985,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea6,
        name_start: 17001,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea7,
        name_start: 17017,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea8,
        name_start: 17033,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ea9,
        name_start: 17048,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eaa,
        name_start: 17063,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eab,
        name_start: 17079,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eac,
        name_start: 17095,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ead,
        name_start: 17114,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eae,
        name_start: 17133,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eaf,
        name_start: 17144,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb0,
        name_start: 17155,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb1,
        name_start: 17166,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb2,
        name_start: 17177,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb3,
        name_start: 17187,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb4,
        name_start: 17197,
        name_len: 11,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb5,
        name_start: 17208,
        name_len: 11,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb6,
        name_start: 17219,
        name_len: 14,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb7,
        name_start: 17233,
        name_len: 14,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb8,
        name_start: 17247,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eb9,
        name_start: 17256,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eba,
        name_start: 17265,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebb,
        name_start: 17270,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebc,
        name_start: 17275,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebd,
        name_start: 17281,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebe,
        name_start: 17287,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ebf,
        name_start: 17303,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec0,
        name_start: 17319,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec1,
        name_start: 17335,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec2,
        name_start: 17351,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec3,
        name_start: 17366,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec4,
        name_start: 17381,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec5,
        name_start: 17397,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec6,
        name_start: 17413,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec7,
        name_start: 17432,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec8,
        name_start: 17451,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ec9,
        name_start: 17456,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eca,
        name_start: 17461,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecb,
        name_start: 17470,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecc,
        name_start: 17479,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecd,
        name_start: 17488,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ece,
        name_start: 17497,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ecf,
        name_start: 17502,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed0,
        name_start: 17507,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed1,
        name_start: 17523,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed2,
        name_start: 17539,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed3,
        name_start: 17555,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed4,
        name_start: 17571,
        name_len: 15,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed5,
        name_start: 17586,
        name_len: 15,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed6,
        name_start: 17601,
        name_len: 16,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed7,
        name_start: 17617,
        name_len: 16,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed8,
        name_start: 17633,
        name_len: 19,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ed9,
        name_start: 17652,
        name_len: 19,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eda,
        name_start: 17671,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edb,
        name_start: 17681,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edc,
        name_start: 17691,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edd,
        name_start: 17701,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ede,
        name_start: 17711,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001edf,
        name_start: 17720,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee0,
        name_start: 17729,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee1,
        name_start: 17739,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee2,
        name_start: 17749,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee3,
        name_start: 17762,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee4,
        name_start: 17775,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee5,
        name_start: 17784,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee6,
        name_start: 17793,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee7,
        name_start: 17798,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee8,
        name_start: 17803,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ee9,
        name_start: 17813,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eea,
        name_start: 17823,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eeb,
        name_start: 17833,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eec,
        name_start: 17843,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eed,
        name_start: 17852,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eee,
        name_start: 17861,
        name_len: 10,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001eef,
        name_start: 17871,
        name_len: 10,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef0,
        name_start: 17881,
        name_len: 13,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef1,
        name_start: 17894,
        name_len: 13,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef4,
        name_start: 17907,
        name_len: 9,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef5,
        name_start: 17916,
        name_len: 9,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef6,
        name_start: 17925,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef7,
        name_start: 17930,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef8,
        name_start: 17935,
        name_len: 6,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01001ef9,
        name_start: 17941,
        name_len: 6,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001a0,
        name_start: 17947,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001a1,
        name_start: 17952,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001af,
        name_start: 17957,
        name_len: 5,
        flags: 0 | IS_UPPER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010001b0,
        name_start: 17962,
        name_len: 5,
        flags: 0 | IS_LOWER | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000303,
        name_start: 17967,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000300,
        name_start: 17982,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000301,
        name_start: 17997,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000309,
        name_start: 18012,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000323,
        name_start: 18026,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a0,
        name_start: 18044,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a1,
        name_start: 18051,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a2,
        name_start: 18060,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a3,
        name_start: 18072,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a4,
        name_start: 18082,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a5,
        name_start: 18090,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a6,
        name_start: 18098,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a7,
        name_start: 18107,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a8,
        name_start: 18117,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020a9,
        name_start: 18126,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020aa,
        name_start: 18133,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010020ab,
        name_start: 18146,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000020ac,
        name_start: 18154,
        name_len: 8,
        flags: 0 | HAS_CHAR | KEYSYM_IS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002070,
        name_start: 18162,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002074,
        name_start: 18174,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002075,
        name_start: 18186,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002076,
        name_start: 18198,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002077,
        name_start: 18209,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002078,
        name_start: 18222,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002079,
        name_start: 18235,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002080,
        name_start: 18247,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002081,
        name_start: 18260,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002082,
        name_start: 18272,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002083,
        name_start: 18284,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002084,
        name_start: 18298,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002085,
        name_start: 18311,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002086,
        name_start: 18324,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002087,
        name_start: 18336,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002088,
        name_start: 18350,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002089,
        name_start: 18364,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002202,
        name_start: 18377,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002205,
        name_start: 18393,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002208,
        name_start: 18401,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002209,
        name_start: 18410,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100220b,
        name_start: 18422,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221a,
        name_start: 18432,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221b,
        name_start: 18442,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100221c,
        name_start: 18450,
        name_len: 10,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100222c,
        name_start: 18460,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100222d,
        name_start: 18469,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002235,
        name_start: 18478,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002248,
        name_start: 18485,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002247,
        name_start: 18493,
        name_len: 11,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002262,
        name_start: 18504,
        name_len: 12,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002263,
        name_start: 18516,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff1,
        name_start: 18524,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff2,
        name_start: 18537,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff3,
        name_start: 18550,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff4,
        name_start: 18563,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff5,
        name_start: 18576,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff6,
        name_start: 18589,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff7,
        name_start: 18602,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff8,
        name_start: 18615,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fff9,
        name_start: 18628,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0000fffa,
        name_start: 18641,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002800,
        name_start: 18655,
        name_len: 13,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002801,
        name_start: 18668,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002802,
        name_start: 18682,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002803,
        name_start: 18696,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002804,
        name_start: 18711,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002805,
        name_start: 18725,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002806,
        name_start: 18740,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002807,
        name_start: 18755,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002808,
        name_start: 18771,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002809,
        name_start: 18785,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280a,
        name_start: 18800,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280b,
        name_start: 18815,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280c,
        name_start: 18831,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280d,
        name_start: 18846,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280e,
        name_start: 18862,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100280f,
        name_start: 18878,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002810,
        name_start: 18895,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002811,
        name_start: 18909,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002812,
        name_start: 18924,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002813,
        name_start: 18939,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002814,
        name_start: 18955,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002815,
        name_start: 18970,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002816,
        name_start: 18986,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002817,
        name_start: 19002,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002818,
        name_start: 19019,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002819,
        name_start: 19034,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281a,
        name_start: 19050,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281b,
        name_start: 19066,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281c,
        name_start: 19083,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281d,
        name_start: 19099,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281e,
        name_start: 19116,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100281f,
        name_start: 19133,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002820,
        name_start: 19151,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002821,
        name_start: 19165,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002822,
        name_start: 19180,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002823,
        name_start: 19195,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002824,
        name_start: 19211,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002825,
        name_start: 19226,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002826,
        name_start: 19242,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002827,
        name_start: 19258,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002828,
        name_start: 19275,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002829,
        name_start: 19290,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282a,
        name_start: 19306,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282b,
        name_start: 19322,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282c,
        name_start: 19339,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282d,
        name_start: 19355,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282e,
        name_start: 19372,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100282f,
        name_start: 19389,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002830,
        name_start: 19407,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002831,
        name_start: 19422,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002832,
        name_start: 19438,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002833,
        name_start: 19454,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002834,
        name_start: 19471,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002835,
        name_start: 19487,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002836,
        name_start: 19504,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002837,
        name_start: 19521,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002838,
        name_start: 19539,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002839,
        name_start: 19555,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283a,
        name_start: 19572,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283b,
        name_start: 19589,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283c,
        name_start: 19607,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283d,
        name_start: 19624,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283e,
        name_start: 19642,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100283f,
        name_start: 19660,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002840,
        name_start: 19679,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002841,
        name_start: 19693,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002842,
        name_start: 19708,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002843,
        name_start: 19723,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002844,
        name_start: 19739,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002845,
        name_start: 19754,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002846,
        name_start: 19770,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002847,
        name_start: 19786,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002848,
        name_start: 19803,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002849,
        name_start: 19818,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284a,
        name_start: 19834,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284b,
        name_start: 19850,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284c,
        name_start: 19867,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284d,
        name_start: 19883,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284e,
        name_start: 19900,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100284f,
        name_start: 19917,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002850,
        name_start: 19935,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002851,
        name_start: 19950,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002852,
        name_start: 19966,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002853,
        name_start: 19982,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002854,
        name_start: 19999,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002855,
        name_start: 20015,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002856,
        name_start: 20032,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002857,
        name_start: 20049,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002858,
        name_start: 20067,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002859,
        name_start: 20083,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285a,
        name_start: 20100,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285b,
        name_start: 20117,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285c,
        name_start: 20135,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285d,
        name_start: 20152,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285e,
        name_start: 20170,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100285f,
        name_start: 20188,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002860,
        name_start: 20207,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002861,
        name_start: 20222,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002862,
        name_start: 20238,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002863,
        name_start: 20254,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002864,
        name_start: 20271,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002865,
        name_start: 20287,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002866,
        name_start: 20304,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002867,
        name_start: 20321,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002868,
        name_start: 20339,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002869,
        name_start: 20355,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286a,
        name_start: 20372,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286b,
        name_start: 20389,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286c,
        name_start: 20407,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286d,
        name_start: 20424,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286e,
        name_start: 20442,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100286f,
        name_start: 20460,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002870,
        name_start: 20479,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002871,
        name_start: 20495,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002872,
        name_start: 20512,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002873,
        name_start: 20529,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002874,
        name_start: 20547,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002875,
        name_start: 20564,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002876,
        name_start: 20582,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002877,
        name_start: 20600,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002878,
        name_start: 20619,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002879,
        name_start: 20636,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287a,
        name_start: 20654,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287b,
        name_start: 20672,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287c,
        name_start: 20691,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287d,
        name_start: 20709,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287e,
        name_start: 20728,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100287f,
        name_start: 20747,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002880,
        name_start: 20767,
        name_len: 14,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002881,
        name_start: 20781,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002882,
        name_start: 20796,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002883,
        name_start: 20811,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002884,
        name_start: 20827,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002885,
        name_start: 20842,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002886,
        name_start: 20858,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002887,
        name_start: 20874,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002888,
        name_start: 20891,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002889,
        name_start: 20906,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288a,
        name_start: 20922,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288b,
        name_start: 20938,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288c,
        name_start: 20955,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288d,
        name_start: 20971,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288e,
        name_start: 20988,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100288f,
        name_start: 21005,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002890,
        name_start: 21023,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002891,
        name_start: 21038,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002892,
        name_start: 21054,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002893,
        name_start: 21070,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002894,
        name_start: 21087,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002895,
        name_start: 21103,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002896,
        name_start: 21120,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002897,
        name_start: 21137,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002898,
        name_start: 21155,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01002899,
        name_start: 21171,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289a,
        name_start: 21188,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289b,
        name_start: 21205,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289c,
        name_start: 21223,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289d,
        name_start: 21240,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289e,
        name_start: 21258,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x0100289f,
        name_start: 21276,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a0,
        name_start: 21295,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a1,
        name_start: 21310,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a2,
        name_start: 21326,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a3,
        name_start: 21342,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a4,
        name_start: 21359,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a5,
        name_start: 21375,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a6,
        name_start: 21392,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a7,
        name_start: 21409,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a8,
        name_start: 21427,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028a9,
        name_start: 21443,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028aa,
        name_start: 21460,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ab,
        name_start: 21477,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ac,
        name_start: 21495,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ad,
        name_start: 21512,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ae,
        name_start: 21530,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028af,
        name_start: 21548,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b0,
        name_start: 21567,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b1,
        name_start: 21583,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b2,
        name_start: 21600,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b3,
        name_start: 21617,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b4,
        name_start: 21635,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b5,
        name_start: 21652,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b6,
        name_start: 21670,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b7,
        name_start: 21688,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b8,
        name_start: 21707,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028b9,
        name_start: 21724,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ba,
        name_start: 21742,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bb,
        name_start: 21760,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bc,
        name_start: 21779,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bd,
        name_start: 21797,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028be,
        name_start: 21816,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028bf,
        name_start: 21835,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c0,
        name_start: 21855,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c1,
        name_start: 21870,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c2,
        name_start: 21886,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c3,
        name_start: 21902,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c4,
        name_start: 21919,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c5,
        name_start: 21935,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c6,
        name_start: 21952,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c7,
        name_start: 21969,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c8,
        name_start: 21987,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028c9,
        name_start: 22003,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ca,
        name_start: 22020,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cb,
        name_start: 22037,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cc,
        name_start: 22055,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cd,
        name_start: 22072,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ce,
        name_start: 22090,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028cf,
        name_start: 22108,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d0,
        name_start: 22127,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d1,
        name_start: 22143,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d2,
        name_start: 22160,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d3,
        name_start: 22177,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d4,
        name_start: 22195,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d5,
        name_start: 22212,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d6,
        name_start: 22230,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d7,
        name_start: 22248,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d8,
        name_start: 22267,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028d9,
        name_start: 22284,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028da,
        name_start: 22302,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028db,
        name_start: 22320,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028dc,
        name_start: 22339,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028dd,
        name_start: 22357,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028de,
        name_start: 22376,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028df,
        name_start: 22395,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e0,
        name_start: 22415,
        name_len: 16,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e1,
        name_start: 22431,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e2,
        name_start: 22448,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e3,
        name_start: 22465,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e4,
        name_start: 22483,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e5,
        name_start: 22500,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e6,
        name_start: 22518,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e7,
        name_start: 22536,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e8,
        name_start: 22555,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028e9,
        name_start: 22572,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ea,
        name_start: 22590,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028eb,
        name_start: 22608,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ec,
        name_start: 22627,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ed,
        name_start: 22645,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ee,
        name_start: 22664,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ef,
        name_start: 22683,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f0,
        name_start: 22703,
        name_len: 17,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f1,
        name_start: 22720,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f2,
        name_start: 22738,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f3,
        name_start: 22756,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f4,
        name_start: 22775,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f5,
        name_start: 22793,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f6,
        name_start: 22812,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f7,
        name_start: 22831,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f8,
        name_start: 22851,
        name_len: 18,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028f9,
        name_start: 22869,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fa,
        name_start: 22888,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fb,
        name_start: 22907,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fc,
        name_start: 22927,
        name_len: 19,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fd,
        name_start: 22946,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028fe,
        name_start: 22966,
        name_len: 20,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x010028ff,
        name_start: 22986,
        name_len: 21,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d82,
        name_start: 23007,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d83,
        name_start: 23014,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d85,
        name_start: 23021,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d86,
        name_start: 23027,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d87,
        name_start: 23034,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d88,
        name_start: 23041,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d89,
        name_start: 23049,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8a,
        name_start: 23055,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8b,
        name_start: 23062,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8c,
        name_start: 23068,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8d,
        name_start: 23075,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8e,
        name_start: 23082,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d8f,
        name_start: 23090,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d90,
        name_start: 23097,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d91,
        name_start: 23105,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d92,
        name_start: 23111,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d93,
        name_start: 23118,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d94,
        name_start: 23125,
        name_len: 6,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d95,
        name_start: 23131,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d96,
        name_start: 23138,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9a,
        name_start: 23145,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9b,
        name_start: 23152,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9c,
        name_start: 23160,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9d,
        name_start: 23167,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9e,
        name_start: 23175,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000d9f,
        name_start: 23183,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da0,
        name_start: 23191,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da1,
        name_start: 23198,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da2,
        name_start: 23206,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da3,
        name_start: 23213,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da4,
        name_start: 23221,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da5,
        name_start: 23229,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da6,
        name_start: 23238,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da7,
        name_start: 23246,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da8,
        name_start: 23254,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000da9,
        name_start: 23263,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000daa,
        name_start: 23271,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dab,
        name_start: 23280,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dac,
        name_start: 23288,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dad,
        name_start: 23297,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dae,
        name_start: 23305,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000daf,
        name_start: 23314,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db0,
        name_start: 23322,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db1,
        name_start: 23331,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db3,
        name_start: 23338,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db4,
        name_start: 23347,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db5,
        name_start: 23354,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db6,
        name_start: 23362,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db7,
        name_start: 23369,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db8,
        name_start: 23377,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000db9,
        name_start: 23384,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dba,
        name_start: 23392,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dbb,
        name_start: 23399,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dbd,
        name_start: 23406,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc0,
        name_start: 23413,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc1,
        name_start: 23420,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc2,
        name_start: 23428,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc3,
        name_start: 23437,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc4,
        name_start: 23444,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc5,
        name_start: 23451,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dc6,
        name_start: 23459,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dca,
        name_start: 23466,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dcf,
        name_start: 23473,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd0,
        name_start: 23481,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd1,
        name_start: 23489,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd2,
        name_start: 23498,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd3,
        name_start: 23505,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd4,
        name_start: 23513,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd6,
        name_start: 23520,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd8,
        name_start: 23528,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dd9,
        name_start: 23536,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dda,
        name_start: 23543,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddb,
        name_start: 23551,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddc,
        name_start: 23559,
        name_len: 7,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddd,
        name_start: 23566,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000dde,
        name_start: 23574,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000ddf,
        name_start: 23582,
        name_len: 8,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df2,
        name_start: 23590,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df3,
        name_start: 23599,
        name_len: 9,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x01000df4,
        name_start: 23608,
        name_len: 15,
        flags: 0 | HAS_CHAR,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff01,
        name_start: 23623,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff02,
        name_start: 23635,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff03,
        name_start: 23654,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff04,
        name_start: 23675,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff05,
        name_start: 23692,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff06,
        name_start: 23711,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff07,
        name_start: 23732,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff10,
        name_start: 23754,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff11,
        name_start: 23765,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff12,
        name_start: 23785,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff13,
        name_start: 23798,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff14,
        name_start: 23818,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff15,
        name_start: 23831,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff16,
        name_start: 23844,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff17,
        name_start: 23857,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff18,
        name_start: 23870,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff19,
        name_start: 23882,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1a,
        name_start: 23890,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1b,
        name_start: 23899,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1c,
        name_start: 23909,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1d,
        name_start: 23924,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1e,
        name_start: 23938,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff1f,
        name_start: 23946,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff20,
        name_start: 23958,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff21,
        name_start: 23970,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff22,
        name_start: 23983,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff23,
        name_start: 24001,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff24,
        name_start: 24013,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff25,
        name_start: 24027,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff26,
        name_start: 24042,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff27,
        name_start: 24050,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff28,
        name_start: 24061,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff29,
        name_start: 24069,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2a,
        name_start: 24080,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2b,
        name_start: 24092,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2c,
        name_start: 24102,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2d,
        name_start: 24111,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2e,
        name_start: 24126,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff2f,
        name_start: 24133,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff30,
        name_start: 24142,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff31,
        name_start: 24155,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff32,
        name_start: 24169,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff33,
        name_start: 24183,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff34,
        name_start: 24197,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff35,
        name_start: 24211,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff36,
        name_start: 24224,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff37,
        name_start: 24232,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff38,
        name_start: 24243,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff39,
        name_start: 24254,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3a,
        name_start: 24269,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3b,
        name_start: 24281,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3c,
        name_start: 24301,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3d,
        name_start: 24312,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3e,
        name_start: 24325,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff3f,
        name_start: 24340,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff40,
        name_start: 24355,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff41,
        name_start: 24366,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff42,
        name_start: 24377,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff43,
        name_start: 24388,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff44,
        name_start: 24399,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff45,
        name_start: 24410,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff46,
        name_start: 24421,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff47,
        name_start: 24432,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff48,
        name_start: 24443,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff49,
        name_start: 24454,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4a,
        name_start: 24465,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4b,
        name_start: 24476,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4c,
        name_start: 24487,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4d,
        name_start: 24498,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4e,
        name_start: 24509,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff4f,
        name_start: 24520,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff50,
        name_start: 24531,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff51,
        name_start: 24550,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff52,
        name_start: 24570,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff53,
        name_start: 24578,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff54,
        name_start: 24584,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff55,
        name_start: 24598,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff56,
        name_start: 24607,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff57,
        name_start: 24616,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff58,
        name_start: 24624,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff59,
        name_start: 24631,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5a,
        name_start: 24642,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5b,
        name_start: 24649,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5c,
        name_start: 24662,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5d,
        name_start: 24671,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5e,
        name_start: 24683,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff5f,
        name_start: 24691,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff60,
        name_start: 24697,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff61,
        name_start: 24707,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff62,
        name_start: 24717,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff63,
        name_start: 24727,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff65,
        name_start: 24738,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff66,
        name_start: 24748,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff67,
        name_start: 24758,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff68,
        name_start: 24769,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff69,
        name_start: 24776,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6a,
        name_start: 24784,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6b,
        name_start: 24798,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6c,
        name_start: 24806,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6d,
        name_start: 24816,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff6e,
        name_start: 24825,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff70,
        name_start: 24834,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff72,
        name_start: 24839,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff73,
        name_start: 24848,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff74,
        name_start: 24858,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff75,
        name_start: 24875,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff76,
        name_start: 24889,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff77,
        name_start: 24903,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff78,
        name_start: 24911,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff79,
        name_start: 24923,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7a,
        name_start: 24937,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7b,
        name_start: 24952,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7c,
        name_start: 24960,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7d,
        name_start: 24969,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7e,
        name_start: 24984,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff7f,
        name_start: 24995,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff80,
        name_start: 25007,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff81,
        name_start: 25019,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff82,
        name_start: 25028,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff84,
        name_start: 25038,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff85,
        name_start: 25048,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff86,
        name_start: 25059,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff87,
        name_start: 25070,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff88,
        name_start: 25079,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff89,
        name_start: 25094,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8a,
        name_start: 25102,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8b,
        name_start: 25110,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8c,
        name_start: 25120,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8d,
        name_start: 25131,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8e,
        name_start: 25139,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff8f,
        name_start: 25152,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff90,
        name_start: 25162,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff91,
        name_start: 25177,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff92,
        name_start: 25189,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff93,
        name_start: 25198,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff94,
        name_start: 25209,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff95,
        name_start: 25222,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff96,
        name_start: 25230,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff97,
        name_start: 25237,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff98,
        name_start: 25253,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff99,
        name_start: 25268,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9a,
        name_start: 25287,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9b,
        name_start: 25299,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9c,
        name_start: 25318,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9d,
        name_start: 25332,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9e,
        name_start: 25345,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ff9f,
        name_start: 25361,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa0,
        name_start: 25369,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa1,
        name_start: 25379,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa2,
        name_start: 25387,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa3,
        name_start: 25398,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa4,
        name_start: 25405,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa5,
        name_start: 25414,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa6,
        name_start: 25424,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa7,
        name_start: 25432,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa8,
        name_start: 25443,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffa9,
        name_start: 25456,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb0,
        name_start: 25474,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb1,
        name_start: 25488,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb2,
        name_start: 25503,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb3,
        name_start: 25519,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb4,
        name_start: 25531,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb5,
        name_start: 25539,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb6,
        name_start: 25549,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb7,
        name_start: 25564,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008ffb8,
        name_start: 25586,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe01,
        name_start: 25600,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe02,
        name_start: 25615,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe03,
        name_start: 25630,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe04,
        name_start: 25645,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe05,
        name_start: 25660,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe06,
        name_start: 25675,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe07,
        name_start: 25690,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe08,
        name_start: 25705,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe09,
        name_start: 25720,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0a,
        name_start: 25735,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0b,
        name_start: 25751,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe0c,
        name_start: 25767,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe20,
        name_start: 25783,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe21,
        name_start: 25793,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe22,
        name_start: 25806,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe23,
        name_start: 25820,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe24,
        name_start: 25834,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008fe25,
        name_start: 25851,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810f4,
        name_start: 25866,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100810f5,
        name_start: 25884,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081166,
        name_start: 25898,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081177,
        name_start: 25906,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081185,
        name_start: 25921,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081188,
        name_start: 25928,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081192,
        name_start: 25937,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081193,
        name_start: 25950,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008119b,
        name_start: 25965,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a0,
        name_start: 25974,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a4,
        name_start: 25988,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a6,
        name_start: 26001,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a8,
        name_start: 26011,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811a9,
        name_start: 26029,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811aa,
        name_start: 26045,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ac,
        name_start: 26057,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ad,
        name_start: 26070,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811af,
        name_start: 26085,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b0,
        name_start: 26102,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b6,
        name_start: 26116,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b7,
        name_start: 26131,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b8,
        name_start: 26146,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811b9,
        name_start: 26162,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811ba,
        name_start: 26180,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bc,
        name_start: 26190,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811bd,
        name_start: 26212,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811be,
        name_start: 26227,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d0,
        name_start: 26242,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811d1,
        name_start: 26248,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100811e5,
        name_start: 26258,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081200,
        name_start: 26274,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081201,
        name_start: 26286,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081202,
        name_start: 26298,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081203,
        name_start: 26310,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081204,
        name_start: 26322,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081205,
        name_start: 26334,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081206,
        name_start: 26346,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081207,
        name_start: 26358,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081208,
        name_start: 26370,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081209,
        name_start: 26382,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120a,
        name_start: 26394,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120b,
        name_start: 26409,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120c,
        name_start: 26425,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120d,
        name_start: 26437,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120e,
        name_start: 26449,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008120f,
        name_start: 26461,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081210,
        name_start: 26473,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081211,
        name_start: 26488,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081215,
        name_start: 26501,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081216,
        name_start: 26517,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081217,
        name_start: 26534,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081218,
        name_start: 26546,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081219,
        name_start: 26560,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121a,
        name_start: 26574,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121b,
        name_start: 26589,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121c,
        name_start: 26604,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121d,
        name_start: 26620,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008121e,
        name_start: 26639,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081230,
        name_start: 26655,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081232,
        name_start: 26668,
        name_len: 21,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081240,
        name_start: 26689,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081241,
        name_start: 26705,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081242,
        name_start: 26720,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081243,
        name_start: 26731,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081244,
        name_start: 26747,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081245,
        name_start: 26760,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081246,
        name_start: 26775,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081247,
        name_start: 26791,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081249,
        name_start: 26804,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124a,
        name_start: 26819,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124b,
        name_start: 26830,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124c,
        name_start: 26852,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124d,
        name_start: 26875,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124e,
        name_start: 26897,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008124f,
        name_start: 26914,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081250,
        name_start: 26930,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081251,
        name_start: 26947,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081260,
        name_start: 26964,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081261,
        name_start: 26986,
        name_len: 22,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081262,
        name_start: 27008,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081263,
        name_start: 27035,
        name_len: 27,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081264,
        name_start: 27062,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081265,
        name_start: 27086,
        name_len: 24,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081266,
        name_start: 27110,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081267,
        name_start: 27121,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081268,
        name_start: 27134,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081269,
        name_start: 27144,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126a,
        name_start: 27156,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126b,
        name_start: 27168,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126c,
        name_start: 27184,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126d,
        name_start: 27197,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126e,
        name_start: 27210,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008126f,
        name_start: 27223,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081270,
        name_start: 27233,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081271,
        name_start: 27249,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081272,
        name_start: 27263,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081273,
        name_start: 27278,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081274,
        name_start: 27285,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081275,
        name_start: 27295,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081276,
        name_start: 27310,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081277,
        name_start: 27325,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081278,
        name_start: 27333,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081279,
        name_start: 27353,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127a,
        name_start: 27376,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127b,
        name_start: 27399,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127c,
        name_start: 27414,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127d,
        name_start: 27433,
        name_len: 25,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127e,
        name_start: 27458,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008127f,
        name_start: 27474,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081280,
        name_start: 27481,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081281,
        name_start: 27493,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081282,
        name_start: 27509,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081283,
        name_start: 27529,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081284,
        name_start: 27547,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081285,
        name_start: 27563,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081286,
        name_start: 27583,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081287,
        name_start: 27599,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081288,
        name_start: 27614,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081290,
        name_start: 27625,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081291,
        name_start: 27635,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081292,
        name_start: 27645,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081293,
        name_start: 27655,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081294,
        name_start: 27665,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081295,
        name_start: 27675,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081296,
        name_start: 27685,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081297,
        name_start: 27695,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081298,
        name_start: 27705,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x10081299,
        name_start: 27715,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129a,
        name_start: 27726,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129b,
        name_start: 27737,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129c,
        name_start: 27748,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129d,
        name_start: 27759,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129e,
        name_start: 27770,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1008129f,
        name_start: 27781,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a0,
        name_start: 27792,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a1,
        name_start: 27803,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a2,
        name_start: 27814,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a3,
        name_start: 27825,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a4,
        name_start: 27836,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a5,
        name_start: 27847,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a6,
        name_start: 27858,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a7,
        name_start: 27869,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a8,
        name_start: 27880,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812a9,
        name_start: 27891,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812aa,
        name_start: 27902,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ab,
        name_start: 27913,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ac,
        name_start: 27924,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ad,
        name_start: 27935,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b0,
        name_start: 27946,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b1,
        name_start: 27966,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b2,
        name_start: 27985,
        name_len: 20,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b3,
        name_start: 28005,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b4,
        name_start: 28021,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b5,
        name_start: 28037,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b8,
        name_start: 28053,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812b9,
        name_start: 28068,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812ba,
        name_start: 28083,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bb,
        name_start: 28098,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100812bc,
        name_start: 28113,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff00,
        name_start: 28128,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff01,
        name_start: 28139,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff02,
        name_start: 28151,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff03,
        name_start: 28162,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff04,
        name_start: 28173,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff05,
        name_start: 28188,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff10,
        name_start: 28201,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff11,
        name_start: 28207,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff60,
        name_start: 28213,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff70,
        name_start: 28223,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff71,
        name_start: 28231,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff72,
        name_start: 28239,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff73,
        name_start: 28246,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff74,
        name_start: 28253,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff75,
        name_start: 28261,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff76,
        name_start: 28267,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff77,
        name_start: 28281,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff78,
        name_start: 28300,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff79,
        name_start: 28312,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7a,
        name_start: 28331,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7b,
        name_start: 28346,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7c,
        name_start: 28369,
        name_len: 23,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1005ff7d,
        name_start: 28392,
        name_len: 19,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000feb0,
        name_start: 28411,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe5e,
        name_start: 28423,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe2c,
        name_start: 28441,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe27,
        name_start: 28456,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe60,
        name_start: 28469,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe7e,
        name_start: 28482,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000fe22,
        name_start: 28488,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff00,
        name_start: 28498,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6f,
        name_start: 28505,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009be,
        name_start: 28516,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff70,
        name_start: 28525,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009c0,
        name_start: 28537,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff71,
        name_start: 28547,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009c2,
        name_start: 28559,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff72,
        name_start: 28569,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009c4,
        name_start: 28581,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff73,
        name_start: 28591,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009c6,
        name_start: 28603,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff74,
        name_start: 28613,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009c8,
        name_start: 28622,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff75,
        name_start: 28629,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ca,
        name_start: 28641,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff48,
        name_start: 28651,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff49,
        name_start: 28662,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6c,
        name_start: 28673,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009ce,
        name_start: 28680,
        name_len: 5,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6d,
        name_start: 28685,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009d0,
        name_start: 28693,
        name_len: 6,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff6e,
        name_start: 28699,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009d2,
        name_start: 28705,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000a8,
        name_start: 28709,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009d4,
        name_start: 28721,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000a9,
        name_start: 28731,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009d6,
        name_start: 28743,
        name_len: 10,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000aa,
        name_start: 28753,
        name_len: 18,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009d8,
        name_start: 28771,
        name_len: 16,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ab,
        name_start: 28787,
        name_len: 16,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009da,
        name_start: 28803,
        name_len: 14,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ac,
        name_start: 28817,
        name_len: 17,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009dc,
        name_start: 28834,
        name_len: 15,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000af,
        name_start: 28849,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009de,
        name_start: 28855,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000be,
        name_start: 28859,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e0,
        name_start: 28868,
        name_len: 7,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000ee,
        name_start: 28875,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e2,
        name_start: 28887,
        name_len: 4,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e2,
        name_start: 28891,
        name_len: 2,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000f6,
        name_start: 28893,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e5,
        name_start: 28904,
        name_len: 9,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x100000fc,
        name_start: 28913,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x000009e7,
        name_start: 28920,
        name_len: 5,
        flags: 0 | IS_SECONDARY_IDX,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff02,
        name_start: 28925,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff03,
        name_start: 28932,
        name_len: 6,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff04,
        name_start: 28938,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff07,
        name_start: 28946,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff08,
        name_start: 28956,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff0b,
        name_start: 28968,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff1b,
        name_start: 28976,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff31,
        name_start: 28985,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff32,
        name_start: 28995,
        name_len: 15,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff33,
        name_start: 29010,
        name_len: 13,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff40,
        name_start: 29023,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff41,
        name_start: 29034,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff42,
        name_start: 29043,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff43,
        name_start: 29054,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff44,
        name_start: 29066,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff45,
        name_start: 29077,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff51,
        name_start: 29087,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff52,
        name_start: 29094,
        name_len: 5,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff53,
        name_start: 29099,
        name_len: 8,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff54,
        name_start: 29107,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff57,
        name_start: 29114,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff58,
        name_start: 29124,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff59,
        name_start: 29136,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5a,
        name_start: 29146,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5b,
        name_start: 29158,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5c,
        name_start: 29169,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5d,
        name_start: 29180,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff5e,
        name_start: 29192,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff60,
        name_start: 29204,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff63,
        name_start: 29213,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff65,
        name_start: 29222,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff67,
        name_start: 29229,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff69,
        name_start: 29236,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff6a,
        name_start: 29245,
        name_len: 7,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff71,
        name_start: 29252,
        name_len: 12,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff72,
        name_start: 29264,
        name_len: 14,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff73,
        name_start: 29278,
        name_len: 11,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff74,
        name_start: 29289,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ff78,
        name_start: 29298,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1004ffff,
        name_start: 29308,
        name_len: 9,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff76,
        name_start: 29317,
        name_len: 10,
        flags: 0,
    },
    KeysymData {
        keysym_or_definitive_idx: 0x1000ff77,
        name_start: 29327,
        name_len: 10,
        flags: 0,
    },
];

pub mod syms {
    #![allow(non_upper_case_globals)]
    use super::*;
    /// NoSymbol
    pub const KEY_NoSymbol: Keysym = Keysym(0x00000000);
    /// VoidSymbol
    pub const KEY_VoidSymbol: Keysym = Keysym(0x00ffffff);
    /// BackSpace
    pub const KEY_BackSpace: Keysym = Keysym(0x0000ff08);
    /// Tab
    pub const KEY_Tab: Keysym = Keysym(0x0000ff09);
    /// Linefeed
    pub const KEY_Linefeed: Keysym = Keysym(0x0000ff0a);
    /// Clear
    pub const KEY_Clear: Keysym = Keysym(0x0000ff0b);
    /// Return
    pub const KEY_Return: Keysym = Keysym(0x0000ff0d);
    /// Pause
    pub const KEY_Pause: Keysym = Keysym(0x0000ff13);
    /// Scroll_Lock
    pub const KEY_Scroll_Lock: Keysym = Keysym(0x0000ff14);
    /// Sys_Req
    pub const KEY_Sys_Req: Keysym = Keysym(0x0000ff15);
    /// Escape
    pub const KEY_Escape: Keysym = Keysym(0x0000ff1b);
    /// Delete
    pub const KEY_Delete: Keysym = Keysym(0x0000ffff);
    /// Multi_key
    pub const KEY_Multi_key: Keysym = Keysym(0x0000ff20);
    /// SunCompose
    pub const KEY_SunCompose: Keysym = Keysym(0x0000ff20);
    /// Codeinput
    pub const KEY_Codeinput: Keysym = Keysym(0x0000ff37);
    /// Kanji_Bangou
    pub const KEY_Kanji_Bangou: Keysym = Keysym(0x0000ff37);
    /// Hangul_Codeinput
    pub const KEY_Hangul_Codeinput: Keysym = Keysym(0x0000ff37);
    /// SingleCandidate
    pub const KEY_SingleCandidate: Keysym = Keysym(0x0000ff3c);
    /// Hangul_SingleCandidate
    pub const KEY_Hangul_SingleCandidate: Keysym = Keysym(0x0000ff3c);
    /// MultipleCandidate
    pub const KEY_MultipleCandidate: Keysym = Keysym(0x0000ff3d);
    /// Zen_Koho
    pub const KEY_Zen_Koho: Keysym = Keysym(0x0000ff3d);
    /// Hangul_MultipleCandidate
    pub const KEY_Hangul_MultipleCandidate: Keysym = Keysym(0x0000ff3d);
    /// PreviousCandidate
    pub const KEY_PreviousCandidate: Keysym = Keysym(0x0000ff3e);
    /// Mae_Koho
    pub const KEY_Mae_Koho: Keysym = Keysym(0x0000ff3e);
    /// Hangul_PreviousCandidate
    pub const KEY_Hangul_PreviousCandidate: Keysym = Keysym(0x0000ff3e);
    /// Kanji
    pub const KEY_Kanji: Keysym = Keysym(0x0000ff21);
    /// Muhenkan
    pub const KEY_Muhenkan: Keysym = Keysym(0x0000ff22);
    /// Henkan_Mode
    pub const KEY_Henkan_Mode: Keysym = Keysym(0x0000ff23);
    /// Henkan
    pub const KEY_Henkan: Keysym = Keysym(0x0000ff23);
    /// Romaji
    pub const KEY_Romaji: Keysym = Keysym(0x0000ff24);
    /// Hiragana
    pub const KEY_Hiragana: Keysym = Keysym(0x0000ff25);
    /// Katakana
    pub const KEY_Katakana: Keysym = Keysym(0x0000ff26);
    /// Hiragana_Katakana
    pub const KEY_Hiragana_Katakana: Keysym = Keysym(0x0000ff27);
    /// Zenkaku
    pub const KEY_Zenkaku: Keysym = Keysym(0x0000ff28);
    /// Hankaku
    pub const KEY_Hankaku: Keysym = Keysym(0x0000ff29);
    /// Zenkaku_Hankaku
    pub const KEY_Zenkaku_Hankaku: Keysym = Keysym(0x0000ff2a);
    /// Touroku
    pub const KEY_Touroku: Keysym = Keysym(0x0000ff2b);
    /// Massyo
    pub const KEY_Massyo: Keysym = Keysym(0x0000ff2c);
    /// Kana_Lock
    pub const KEY_Kana_Lock: Keysym = Keysym(0x0000ff2d);
    /// Kana_Shift
    pub const KEY_Kana_Shift: Keysym = Keysym(0x0000ff2e);
    /// Eisu_Shift
    pub const KEY_Eisu_Shift: Keysym = Keysym(0x0000ff2f);
    /// Eisu_toggle
    pub const KEY_Eisu_toggle: Keysym = Keysym(0x0000ff30);
    /// Home
    pub const KEY_Home: Keysym = Keysym(0x0000ff50);
    /// Left
    pub const KEY_Left: Keysym = Keysym(0x0000ff51);
    /// Up
    pub const KEY_Up: Keysym = Keysym(0x0000ff52);
    /// Right
    pub const KEY_Right: Keysym = Keysym(0x0000ff53);
    /// Down
    pub const KEY_Down: Keysym = Keysym(0x0000ff54);
    /// Prior
    pub const KEY_Prior: Keysym = Keysym(0x0000ff55);
    /// Page_Up
    pub const KEY_Page_Up: Keysym = Keysym(0x0000ff55);
    /// SunPageUp
    pub const KEY_SunPageUp: Keysym = Keysym(0x0000ff55);
    /// Next
    pub const KEY_Next: Keysym = Keysym(0x0000ff56);
    /// Page_Down
    pub const KEY_Page_Down: Keysym = Keysym(0x0000ff56);
    /// SunPageDown
    pub const KEY_SunPageDown: Keysym = Keysym(0x0000ff56);
    /// End
    pub const KEY_End: Keysym = Keysym(0x0000ff57);
    /// Begin
    pub const KEY_Begin: Keysym = Keysym(0x0000ff58);
    /// Select
    pub const KEY_Select: Keysym = Keysym(0x0000ff60);
    /// Print
    pub const KEY_Print: Keysym = Keysym(0x0000ff61);
    /// SunPrint_Screen
    pub const KEY_SunPrint_Screen: Keysym = Keysym(0x0000ff61);
    /// Execute
    pub const KEY_Execute: Keysym = Keysym(0x0000ff62);
    /// Insert
    pub const KEY_Insert: Keysym = Keysym(0x0000ff63);
    /// Undo
    pub const KEY_Undo: Keysym = Keysym(0x0000ff65);
    /// SunUndo
    pub const KEY_SunUndo: Keysym = Keysym(0x0000ff65);
    /// Redo
    pub const KEY_Redo: Keysym = Keysym(0x0000ff66);
    /// SunAgain
    pub const KEY_SunAgain: Keysym = Keysym(0x0000ff66);
    /// Menu
    pub const KEY_Menu: Keysym = Keysym(0x0000ff67);
    /// Find
    pub const KEY_Find: Keysym = Keysym(0x0000ff68);
    /// SunFind
    pub const KEY_SunFind: Keysym = Keysym(0x0000ff68);
    /// Cancel
    pub const KEY_Cancel: Keysym = Keysym(0x0000ff69);
    /// SunStop
    pub const KEY_SunStop: Keysym = Keysym(0x0000ff69);
    /// Help
    pub const KEY_Help: Keysym = Keysym(0x0000ff6a);
    /// Break
    pub const KEY_Break: Keysym = Keysym(0x0000ff6b);
    /// Mode_switch
    pub const KEY_Mode_switch: Keysym = Keysym(0x0000ff7e);
    /// script_switch
    pub const KEY_script_switch: Keysym = Keysym(0x0000ff7e);
    /// ISO_Group_Shift
    pub const KEY_ISO_Group_Shift: Keysym = Keysym(0x0000ff7e);
    /// kana_switch
    pub const KEY_kana_switch: Keysym = Keysym(0x0000ff7e);
    /// Arabic_switch
    pub const KEY_Arabic_switch: Keysym = Keysym(0x0000ff7e);
    /// Greek_switch
    pub const KEY_Greek_switch: Keysym = Keysym(0x0000ff7e);
    /// Hebrew_switch
    pub const KEY_Hebrew_switch: Keysym = Keysym(0x0000ff7e);
    /// Hangul_switch
    pub const KEY_Hangul_switch: Keysym = Keysym(0x0000ff7e);
    /// SunAltGraph
    pub const KEY_SunAltGraph: Keysym = Keysym(0x0000ff7e);
    /// Num_Lock
    pub const KEY_Num_Lock: Keysym = Keysym(0x0000ff7f);
    /// KP_Space
    pub const KEY_KP_Space: Keysym = Keysym(0x0000ff80);
    /// KP_Tab
    pub const KEY_KP_Tab: Keysym = Keysym(0x0000ff89);
    /// KP_Enter
    pub const KEY_KP_Enter: Keysym = Keysym(0x0000ff8d);
    /// KP_F1
    pub const KEY_KP_F1: Keysym = Keysym(0x0000ff91);
    /// KP_F2
    pub const KEY_KP_F2: Keysym = Keysym(0x0000ff92);
    /// KP_F3
    pub const KEY_KP_F3: Keysym = Keysym(0x0000ff93);
    /// KP_F4
    pub const KEY_KP_F4: Keysym = Keysym(0x0000ff94);
    /// KP_Home
    pub const KEY_KP_Home: Keysym = Keysym(0x0000ff95);
    /// KP_Left
    pub const KEY_KP_Left: Keysym = Keysym(0x0000ff96);
    /// KP_Up
    pub const KEY_KP_Up: Keysym = Keysym(0x0000ff97);
    /// KP_Right
    pub const KEY_KP_Right: Keysym = Keysym(0x0000ff98);
    /// KP_Down
    pub const KEY_KP_Down: Keysym = Keysym(0x0000ff99);
    /// KP_Prior
    pub const KEY_KP_Prior: Keysym = Keysym(0x0000ff9a);
    /// KP_Page_Up
    pub const KEY_KP_Page_Up: Keysym = Keysym(0x0000ff9a);
    /// KP_Next
    pub const KEY_KP_Next: Keysym = Keysym(0x0000ff9b);
    /// KP_Page_Down
    pub const KEY_KP_Page_Down: Keysym = Keysym(0x0000ff9b);
    /// KP_End
    pub const KEY_KP_End: Keysym = Keysym(0x0000ff9c);
    /// KP_Begin
    pub const KEY_KP_Begin: Keysym = Keysym(0x0000ff9d);
    /// KP_Insert
    pub const KEY_KP_Insert: Keysym = Keysym(0x0000ff9e);
    /// KP_Delete
    pub const KEY_KP_Delete: Keysym = Keysym(0x0000ff9f);
    /// KP_Equal
    pub const KEY_KP_Equal: Keysym = Keysym(0x0000ffbd);
    /// KP_Multiply
    pub const KEY_KP_Multiply: Keysym = Keysym(0x0000ffaa);
    /// KP_Add
    pub const KEY_KP_Add: Keysym = Keysym(0x0000ffab);
    /// KP_Separator
    pub const KEY_KP_Separator: Keysym = Keysym(0x0000ffac);
    /// KP_Subtract
    pub const KEY_KP_Subtract: Keysym = Keysym(0x0000ffad);
    /// KP_Decimal
    pub const KEY_KP_Decimal: Keysym = Keysym(0x0000ffae);
    /// KP_Divide
    pub const KEY_KP_Divide: Keysym = Keysym(0x0000ffaf);
    /// KP_0
    pub const KEY_KP_0: Keysym = Keysym(0x0000ffb0);
    /// KP_1
    pub const KEY_KP_1: Keysym = Keysym(0x0000ffb1);
    /// KP_2
    pub const KEY_KP_2: Keysym = Keysym(0x0000ffb2);
    /// KP_3
    pub const KEY_KP_3: Keysym = Keysym(0x0000ffb3);
    /// KP_4
    pub const KEY_KP_4: Keysym = Keysym(0x0000ffb4);
    /// KP_5
    pub const KEY_KP_5: Keysym = Keysym(0x0000ffb5);
    /// KP_6
    pub const KEY_KP_6: Keysym = Keysym(0x0000ffb6);
    /// KP_7
    pub const KEY_KP_7: Keysym = Keysym(0x0000ffb7);
    /// KP_8
    pub const KEY_KP_8: Keysym = Keysym(0x0000ffb8);
    /// KP_9
    pub const KEY_KP_9: Keysym = Keysym(0x0000ffb9);
    /// F1
    pub const KEY_F1: Keysym = Keysym(0x0000ffbe);
    /// F2
    pub const KEY_F2: Keysym = Keysym(0x0000ffbf);
    /// F3
    pub const KEY_F3: Keysym = Keysym(0x0000ffc0);
    /// F4
    pub const KEY_F4: Keysym = Keysym(0x0000ffc1);
    /// F5
    pub const KEY_F5: Keysym = Keysym(0x0000ffc2);
    /// F6
    pub const KEY_F6: Keysym = Keysym(0x0000ffc3);
    /// F7
    pub const KEY_F7: Keysym = Keysym(0x0000ffc4);
    /// F8
    pub const KEY_F8: Keysym = Keysym(0x0000ffc5);
    /// F9
    pub const KEY_F9: Keysym = Keysym(0x0000ffc6);
    /// F10
    pub const KEY_F10: Keysym = Keysym(0x0000ffc7);
    /// F11
    pub const KEY_F11: Keysym = Keysym(0x0000ffc8);
    /// L1
    pub const KEY_L1: Keysym = Keysym(0x0000ffc8);
    /// F12
    pub const KEY_F12: Keysym = Keysym(0x0000ffc9);
    /// L2
    pub const KEY_L2: Keysym = Keysym(0x0000ffc9);
    /// F13
    pub const KEY_F13: Keysym = Keysym(0x0000ffca);
    /// L3
    pub const KEY_L3: Keysym = Keysym(0x0000ffca);
    /// F14
    pub const KEY_F14: Keysym = Keysym(0x0000ffcb);
    /// L4
    pub const KEY_L4: Keysym = Keysym(0x0000ffcb);
    /// F15
    pub const KEY_F15: Keysym = Keysym(0x0000ffcc);
    /// L5
    pub const KEY_L5: Keysym = Keysym(0x0000ffcc);
    /// F16
    pub const KEY_F16: Keysym = Keysym(0x0000ffcd);
    /// L6
    pub const KEY_L6: Keysym = Keysym(0x0000ffcd);
    /// F17
    pub const KEY_F17: Keysym = Keysym(0x0000ffce);
    /// L7
    pub const KEY_L7: Keysym = Keysym(0x0000ffce);
    /// F18
    pub const KEY_F18: Keysym = Keysym(0x0000ffcf);
    /// L8
    pub const KEY_L8: Keysym = Keysym(0x0000ffcf);
    /// F19
    pub const KEY_F19: Keysym = Keysym(0x0000ffd0);
    /// L9
    pub const KEY_L9: Keysym = Keysym(0x0000ffd0);
    /// F20
    pub const KEY_F20: Keysym = Keysym(0x0000ffd1);
    /// L10
    pub const KEY_L10: Keysym = Keysym(0x0000ffd1);
    /// F21
    pub const KEY_F21: Keysym = Keysym(0x0000ffd2);
    /// R1
    pub const KEY_R1: Keysym = Keysym(0x0000ffd2);
    /// F22
    pub const KEY_F22: Keysym = Keysym(0x0000ffd3);
    /// R2
    pub const KEY_R2: Keysym = Keysym(0x0000ffd3);
    /// F23
    pub const KEY_F23: Keysym = Keysym(0x0000ffd4);
    /// R3
    pub const KEY_R3: Keysym = Keysym(0x0000ffd4);
    /// F24
    pub const KEY_F24: Keysym = Keysym(0x0000ffd5);
    /// R4
    pub const KEY_R4: Keysym = Keysym(0x0000ffd5);
    /// F25
    pub const KEY_F25: Keysym = Keysym(0x0000ffd6);
    /// R5
    pub const KEY_R5: Keysym = Keysym(0x0000ffd6);
    /// F26
    pub const KEY_F26: Keysym = Keysym(0x0000ffd7);
    /// R6
    pub const KEY_R6: Keysym = Keysym(0x0000ffd7);
    /// F27
    pub const KEY_F27: Keysym = Keysym(0x0000ffd8);
    /// R7
    pub const KEY_R7: Keysym = Keysym(0x0000ffd8);
    /// F28
    pub const KEY_F28: Keysym = Keysym(0x0000ffd9);
    /// R8
    pub const KEY_R8: Keysym = Keysym(0x0000ffd9);
    /// F29
    pub const KEY_F29: Keysym = Keysym(0x0000ffda);
    /// R9
    pub const KEY_R9: Keysym = Keysym(0x0000ffda);
    /// F30
    pub const KEY_F30: Keysym = Keysym(0x0000ffdb);
    /// R10
    pub const KEY_R10: Keysym = Keysym(0x0000ffdb);
    /// F31
    pub const KEY_F31: Keysym = Keysym(0x0000ffdc);
    /// R11
    pub const KEY_R11: Keysym = Keysym(0x0000ffdc);
    /// F32
    pub const KEY_F32: Keysym = Keysym(0x0000ffdd);
    /// R12
    pub const KEY_R12: Keysym = Keysym(0x0000ffdd);
    /// F33
    pub const KEY_F33: Keysym = Keysym(0x0000ffde);
    /// R13
    pub const KEY_R13: Keysym = Keysym(0x0000ffde);
    /// F34
    pub const KEY_F34: Keysym = Keysym(0x0000ffdf);
    /// R14
    pub const KEY_R14: Keysym = Keysym(0x0000ffdf);
    /// F35
    pub const KEY_F35: Keysym = Keysym(0x0000ffe0);
    /// R15
    pub const KEY_R15: Keysym = Keysym(0x0000ffe0);
    /// Shift_L
    pub const KEY_Shift_L: Keysym = Keysym(0x0000ffe1);
    /// Shift_R
    pub const KEY_Shift_R: Keysym = Keysym(0x0000ffe2);
    /// Control_L
    pub const KEY_Control_L: Keysym = Keysym(0x0000ffe3);
    /// Control_R
    pub const KEY_Control_R: Keysym = Keysym(0x0000ffe4);
    /// Caps_Lock
    pub const KEY_Caps_Lock: Keysym = Keysym(0x0000ffe5);
    /// Shift_Lock
    pub const KEY_Shift_Lock: Keysym = Keysym(0x0000ffe6);
    /// Meta_L
    pub const KEY_Meta_L: Keysym = Keysym(0x0000ffe7);
    /// Meta_R
    pub const KEY_Meta_R: Keysym = Keysym(0x0000ffe8);
    /// Alt_L
    pub const KEY_Alt_L: Keysym = Keysym(0x0000ffe9);
    /// Alt_R
    pub const KEY_Alt_R: Keysym = Keysym(0x0000ffea);
    /// Super_L
    pub const KEY_Super_L: Keysym = Keysym(0x0000ffeb);
    /// Super_R
    pub const KEY_Super_R: Keysym = Keysym(0x0000ffec);
    /// Hyper_L
    pub const KEY_Hyper_L: Keysym = Keysym(0x0000ffed);
    /// Hyper_R
    pub const KEY_Hyper_R: Keysym = Keysym(0x0000ffee);
    /// ISO_Lock
    pub const KEY_ISO_Lock: Keysym = Keysym(0x0000fe01);
    /// ISO_Level2_Latch
    pub const KEY_ISO_Level2_Latch: Keysym = Keysym(0x0000fe02);
    /// ISO_Level3_Shift
    pub const KEY_ISO_Level3_Shift: Keysym = Keysym(0x0000fe03);
    /// ISO_Level3_Latch
    pub const KEY_ISO_Level3_Latch: Keysym = Keysym(0x0000fe04);
    /// ISO_Level3_Lock
    pub const KEY_ISO_Level3_Lock: Keysym = Keysym(0x0000fe05);
    /// ISO_Level5_Shift
    pub const KEY_ISO_Level5_Shift: Keysym = Keysym(0x0000fe11);
    /// ISO_Level5_Latch
    pub const KEY_ISO_Level5_Latch: Keysym = Keysym(0x0000fe12);
    /// ISO_Level5_Lock
    pub const KEY_ISO_Level5_Lock: Keysym = Keysym(0x0000fe13);
    /// ISO_Group_Latch
    pub const KEY_ISO_Group_Latch: Keysym = Keysym(0x0000fe06);
    /// ISO_Group_Lock
    pub const KEY_ISO_Group_Lock: Keysym = Keysym(0x0000fe07);
    /// ISO_Next_Group
    pub const KEY_ISO_Next_Group: Keysym = Keysym(0x0000fe08);
    /// ISO_Next_Group_Lock
    pub const KEY_ISO_Next_Group_Lock: Keysym = Keysym(0x0000fe09);
    /// ISO_Prev_Group
    pub const KEY_ISO_Prev_Group: Keysym = Keysym(0x0000fe0a);
    /// ISO_Prev_Group_Lock
    pub const KEY_ISO_Prev_Group_Lock: Keysym = Keysym(0x0000fe0b);
    /// ISO_First_Group
    pub const KEY_ISO_First_Group: Keysym = Keysym(0x0000fe0c);
    /// ISO_First_Group_Lock
    pub const KEY_ISO_First_Group_Lock: Keysym = Keysym(0x0000fe0d);
    /// ISO_Last_Group
    pub const KEY_ISO_Last_Group: Keysym = Keysym(0x0000fe0e);
    /// ISO_Last_Group_Lock
    pub const KEY_ISO_Last_Group_Lock: Keysym = Keysym(0x0000fe0f);
    /// ISO_Left_Tab
    pub const KEY_ISO_Left_Tab: Keysym = Keysym(0x0000fe20);
    /// ISO_Move_Line_Up
    pub const KEY_ISO_Move_Line_Up: Keysym = Keysym(0x0000fe21);
    /// ISO_Move_Line_Down
    pub const KEY_ISO_Move_Line_Down: Keysym = Keysym(0x0000fe22);
    /// ISO_Partial_Line_Up
    pub const KEY_ISO_Partial_Line_Up: Keysym = Keysym(0x0000fe23);
    /// ISO_Partial_Line_Down
    pub const KEY_ISO_Partial_Line_Down: Keysym = Keysym(0x0000fe24);
    /// ISO_Partial_Space_Left
    pub const KEY_ISO_Partial_Space_Left: Keysym = Keysym(0x0000fe25);
    /// ISO_Partial_Space_Right
    pub const KEY_ISO_Partial_Space_Right: Keysym = Keysym(0x0000fe26);
    /// ISO_Set_Margin_Left
    pub const KEY_ISO_Set_Margin_Left: Keysym = Keysym(0x0000fe27);
    /// ISO_Set_Margin_Right
    pub const KEY_ISO_Set_Margin_Right: Keysym = Keysym(0x0000fe28);
    /// ISO_Release_Margin_Left
    pub const KEY_ISO_Release_Margin_Left: Keysym = Keysym(0x0000fe29);
    /// ISO_Release_Margin_Right
    pub const KEY_ISO_Release_Margin_Right: Keysym = Keysym(0x0000fe2a);
    /// ISO_Release_Both_Margins
    pub const KEY_ISO_Release_Both_Margins: Keysym = Keysym(0x0000fe2b);
    /// ISO_Fast_Cursor_Left
    pub const KEY_ISO_Fast_Cursor_Left: Keysym = Keysym(0x0000fe2c);
    /// ISO_Fast_Cursor_Right
    pub const KEY_ISO_Fast_Cursor_Right: Keysym = Keysym(0x0000fe2d);
    /// ISO_Fast_Cursor_Up
    pub const KEY_ISO_Fast_Cursor_Up: Keysym = Keysym(0x0000fe2e);
    /// ISO_Fast_Cursor_Down
    pub const KEY_ISO_Fast_Cursor_Down: Keysym = Keysym(0x0000fe2f);
    /// ISO_Continuous_Underline
    pub const KEY_ISO_Continuous_Underline: Keysym = Keysym(0x0000fe30);
    /// ISO_Discontinuous_Underline
    pub const KEY_ISO_Discontinuous_Underline: Keysym = Keysym(0x0000fe31);
    /// ISO_Emphasize
    pub const KEY_ISO_Emphasize: Keysym = Keysym(0x0000fe32);
    /// ISO_Center_Object
    pub const KEY_ISO_Center_Object: Keysym = Keysym(0x0000fe33);
    /// ISO_Enter
    pub const KEY_ISO_Enter: Keysym = Keysym(0x0000fe34);
    /// dead_grave
    pub const KEY_dead_grave: Keysym = Keysym(0x0000fe50);
    /// dead_acute
    pub const KEY_dead_acute: Keysym = Keysym(0x0000fe51);
    /// dead_circumflex
    pub const KEY_dead_circumflex: Keysym = Keysym(0x0000fe52);
    /// dead_tilde
    pub const KEY_dead_tilde: Keysym = Keysym(0x0000fe53);
    /// dead_perispomeni
    pub const KEY_dead_perispomeni: Keysym = Keysym(0x0000fe53);
    /// dead_macron
    pub const KEY_dead_macron: Keysym = Keysym(0x0000fe54);
    /// dead_breve
    pub const KEY_dead_breve: Keysym = Keysym(0x0000fe55);
    /// dead_abovedot
    pub const KEY_dead_abovedot: Keysym = Keysym(0x0000fe56);
    /// dead_diaeresis
    pub const KEY_dead_diaeresis: Keysym = Keysym(0x0000fe57);
    /// dead_abovering
    pub const KEY_dead_abovering: Keysym = Keysym(0x0000fe58);
    /// dead_doubleacute
    pub const KEY_dead_doubleacute: Keysym = Keysym(0x0000fe59);
    /// dead_caron
    pub const KEY_dead_caron: Keysym = Keysym(0x0000fe5a);
    /// dead_cedilla
    pub const KEY_dead_cedilla: Keysym = Keysym(0x0000fe5b);
    /// dead_ogonek
    pub const KEY_dead_ogonek: Keysym = Keysym(0x0000fe5c);
    /// dead_iota
    pub const KEY_dead_iota: Keysym = Keysym(0x0000fe5d);
    /// dead_voiced_sound
    pub const KEY_dead_voiced_sound: Keysym = Keysym(0x0000fe5e);
    /// dead_semivoiced_sound
    pub const KEY_dead_semivoiced_sound: Keysym = Keysym(0x0000fe5f);
    /// dead_belowdot
    pub const KEY_dead_belowdot: Keysym = Keysym(0x0000fe60);
    /// dead_hook
    pub const KEY_dead_hook: Keysym = Keysym(0x0000fe61);
    /// dead_horn
    pub const KEY_dead_horn: Keysym = Keysym(0x0000fe62);
    /// dead_stroke
    pub const KEY_dead_stroke: Keysym = Keysym(0x0000fe63);
    /// dead_abovecomma
    pub const KEY_dead_abovecomma: Keysym = Keysym(0x0000fe64);
    /// dead_psili
    pub const KEY_dead_psili: Keysym = Keysym(0x0000fe64);
    /// dead_abovereversedcomma
    pub const KEY_dead_abovereversedcomma: Keysym = Keysym(0x0000fe65);
    /// dead_dasia
    pub const KEY_dead_dasia: Keysym = Keysym(0x0000fe65);
    /// dead_doublegrave
    pub const KEY_dead_doublegrave: Keysym = Keysym(0x0000fe66);
    /// dead_belowring
    pub const KEY_dead_belowring: Keysym = Keysym(0x0000fe67);
    /// dead_belowmacron
    pub const KEY_dead_belowmacron: Keysym = Keysym(0x0000fe68);
    /// dead_belowcircumflex
    pub const KEY_dead_belowcircumflex: Keysym = Keysym(0x0000fe69);
    /// dead_belowtilde
    pub const KEY_dead_belowtilde: Keysym = Keysym(0x0000fe6a);
    /// dead_belowbreve
    pub const KEY_dead_belowbreve: Keysym = Keysym(0x0000fe6b);
    /// dead_belowdiaeresis
    pub const KEY_dead_belowdiaeresis: Keysym = Keysym(0x0000fe6c);
    /// dead_invertedbreve
    pub const KEY_dead_invertedbreve: Keysym = Keysym(0x0000fe6d);
    /// dead_belowcomma
    pub const KEY_dead_belowcomma: Keysym = Keysym(0x0000fe6e);
    /// dead_currency
    pub const KEY_dead_currency: Keysym = Keysym(0x0000fe6f);
    /// dead_lowline
    pub const KEY_dead_lowline: Keysym = Keysym(0x0000fe90);
    /// dead_aboveverticalline
    pub const KEY_dead_aboveverticalline: Keysym = Keysym(0x0000fe91);
    /// dead_belowverticalline
    pub const KEY_dead_belowverticalline: Keysym = Keysym(0x0000fe92);
    /// dead_longsolidusoverlay
    pub const KEY_dead_longsolidusoverlay: Keysym = Keysym(0x0000fe93);
    /// dead_a
    pub const KEY_dead_a: Keysym = Keysym(0x0000fe80);
    /// dead_A
    pub const KEY_dead_A: Keysym = Keysym(0x0000fe81);
    /// dead_e
    pub const KEY_dead_e: Keysym = Keysym(0x0000fe82);
    /// dead_E
    pub const KEY_dead_E: Keysym = Keysym(0x0000fe83);
    /// dead_i
    pub const KEY_dead_i: Keysym = Keysym(0x0000fe84);
    /// dead_I
    pub const KEY_dead_I: Keysym = Keysym(0x0000fe85);
    /// dead_o
    pub const KEY_dead_o: Keysym = Keysym(0x0000fe86);
    /// dead_O
    pub const KEY_dead_O: Keysym = Keysym(0x0000fe87);
    /// dead_u
    pub const KEY_dead_u: Keysym = Keysym(0x0000fe88);
    /// dead_U
    pub const KEY_dead_U: Keysym = Keysym(0x0000fe89);
    /// dead_small_schwa
    pub const KEY_dead_small_schwa: Keysym = Keysym(0x0000fe8a);
    /// dead_schwa
    pub const KEY_dead_schwa: Keysym = Keysym(0x0000fe8a);
    /// dead_capital_schwa
    pub const KEY_dead_capital_schwa: Keysym = Keysym(0x0000fe8b);
    /// dead_SCHWA
    pub const KEY_dead_SCHWA: Keysym = Keysym(0x0000fe8b);
    /// dead_greek
    pub const KEY_dead_greek: Keysym = Keysym(0x0000fe8c);
    /// dead_hamza
    pub const KEY_dead_hamza: Keysym = Keysym(0x0000fe8d);
    /// First_Virtual_Screen
    pub const KEY_First_Virtual_Screen: Keysym = Keysym(0x0000fed0);
    /// Prev_Virtual_Screen
    pub const KEY_Prev_Virtual_Screen: Keysym = Keysym(0x0000fed1);
    /// Next_Virtual_Screen
    pub const KEY_Next_Virtual_Screen: Keysym = Keysym(0x0000fed2);
    /// Last_Virtual_Screen
    pub const KEY_Last_Virtual_Screen: Keysym = Keysym(0x0000fed4);
    /// Terminate_Server
    pub const KEY_Terminate_Server: Keysym = Keysym(0x0000fed5);
    /// AccessX_Enable
    pub const KEY_AccessX_Enable: Keysym = Keysym(0x0000fe70);
    /// AccessX_Feedback_Enable
    pub const KEY_AccessX_Feedback_Enable: Keysym = Keysym(0x0000fe71);
    /// RepeatKeys_Enable
    pub const KEY_RepeatKeys_Enable: Keysym = Keysym(0x0000fe72);
    /// SlowKeys_Enable
    pub const KEY_SlowKeys_Enable: Keysym = Keysym(0x0000fe73);
    /// BounceKeys_Enable
    pub const KEY_BounceKeys_Enable: Keysym = Keysym(0x0000fe74);
    /// StickyKeys_Enable
    pub const KEY_StickyKeys_Enable: Keysym = Keysym(0x0000fe75);
    /// MouseKeys_Enable
    pub const KEY_MouseKeys_Enable: Keysym = Keysym(0x0000fe76);
    /// MouseKeys_Accel_Enable
    pub const KEY_MouseKeys_Accel_Enable: Keysym = Keysym(0x0000fe77);
    /// Overlay1_Enable
    pub const KEY_Overlay1_Enable: Keysym = Keysym(0x0000fe78);
    /// Overlay2_Enable
    pub const KEY_Overlay2_Enable: Keysym = Keysym(0x0000fe79);
    /// AudibleBell_Enable
    pub const KEY_AudibleBell_Enable: Keysym = Keysym(0x0000fe7a);
    /// Pointer_Left
    pub const KEY_Pointer_Left: Keysym = Keysym(0x0000fee0);
    /// Pointer_Right
    pub const KEY_Pointer_Right: Keysym = Keysym(0x0000fee1);
    /// Pointer_Up
    pub const KEY_Pointer_Up: Keysym = Keysym(0x0000fee2);
    /// Pointer_Down
    pub const KEY_Pointer_Down: Keysym = Keysym(0x0000fee3);
    /// Pointer_UpLeft
    pub const KEY_Pointer_UpLeft: Keysym = Keysym(0x0000fee4);
    /// Pointer_UpRight
    pub const KEY_Pointer_UpRight: Keysym = Keysym(0x0000fee5);
    /// Pointer_DownLeft
    pub const KEY_Pointer_DownLeft: Keysym = Keysym(0x0000fee6);
    /// Pointer_DownRight
    pub const KEY_Pointer_DownRight: Keysym = Keysym(0x0000fee7);
    /// Pointer_Button_Dflt
    pub const KEY_Pointer_Button_Dflt: Keysym = Keysym(0x0000fee8);
    /// Pointer_Button1
    pub const KEY_Pointer_Button1: Keysym = Keysym(0x0000fee9);
    /// Pointer_Button2
    pub const KEY_Pointer_Button2: Keysym = Keysym(0x0000feea);
    /// Pointer_Button3
    pub const KEY_Pointer_Button3: Keysym = Keysym(0x0000feeb);
    /// Pointer_Button4
    pub const KEY_Pointer_Button4: Keysym = Keysym(0x0000feec);
    /// Pointer_Button5
    pub const KEY_Pointer_Button5: Keysym = Keysym(0x0000feed);
    /// Pointer_DblClick_Dflt
    pub const KEY_Pointer_DblClick_Dflt: Keysym = Keysym(0x0000feee);
    /// Pointer_DblClick1
    pub const KEY_Pointer_DblClick1: Keysym = Keysym(0x0000feef);
    /// Pointer_DblClick2
    pub const KEY_Pointer_DblClick2: Keysym = Keysym(0x0000fef0);
    /// Pointer_DblClick3
    pub const KEY_Pointer_DblClick3: Keysym = Keysym(0x0000fef1);
    /// Pointer_DblClick4
    pub const KEY_Pointer_DblClick4: Keysym = Keysym(0x0000fef2);
    /// Pointer_DblClick5
    pub const KEY_Pointer_DblClick5: Keysym = Keysym(0x0000fef3);
    /// Pointer_Drag_Dflt
    pub const KEY_Pointer_Drag_Dflt: Keysym = Keysym(0x0000fef4);
    /// Pointer_Drag1
    pub const KEY_Pointer_Drag1: Keysym = Keysym(0x0000fef5);
    /// Pointer_Drag2
    pub const KEY_Pointer_Drag2: Keysym = Keysym(0x0000fef6);
    /// Pointer_Drag3
    pub const KEY_Pointer_Drag3: Keysym = Keysym(0x0000fef7);
    /// Pointer_Drag4
    pub const KEY_Pointer_Drag4: Keysym = Keysym(0x0000fef8);
    /// Pointer_Drag5
    pub const KEY_Pointer_Drag5: Keysym = Keysym(0x0000fefd);
    /// Pointer_EnableKeys
    pub const KEY_Pointer_EnableKeys: Keysym = Keysym(0x0000fef9);
    /// Pointer_Accelerate
    pub const KEY_Pointer_Accelerate: Keysym = Keysym(0x0000fefa);
    /// Pointer_DfltBtnNext
    pub const KEY_Pointer_DfltBtnNext: Keysym = Keysym(0x0000fefb);
    /// Pointer_DfltBtnPrev
    pub const KEY_Pointer_DfltBtnPrev: Keysym = Keysym(0x0000fefc);
    /// ch
    pub const KEY_ch: Keysym = Keysym(0x0000fea0);
    /// Ch
    pub const KEY_Ch: Keysym = Keysym(0x0000fea1);
    /// CH
    pub const KEY_CH: Keysym = Keysym(0x0000fea2);
    /// c_h
    pub const KEY_c_h: Keysym = Keysym(0x0000fea3);
    /// C_h
    pub const KEY_C_h: Keysym = Keysym(0x0000fea4);
    /// C_H
    pub const KEY_C_H: Keysym = Keysym(0x0000fea5);
    /// 3270_Duplicate
    pub const KEY_3270_Duplicate: Keysym = Keysym(0x0000fd01);
    /// 3270_FieldMark
    pub const KEY_3270_FieldMark: Keysym = Keysym(0x0000fd02);
    /// 3270_Right2
    pub const KEY_3270_Right2: Keysym = Keysym(0x0000fd03);
    /// 3270_Left2
    pub const KEY_3270_Left2: Keysym = Keysym(0x0000fd04);
    /// 3270_BackTab
    pub const KEY_3270_BackTab: Keysym = Keysym(0x0000fd05);
    /// 3270_EraseEOF
    pub const KEY_3270_EraseEOF: Keysym = Keysym(0x0000fd06);
    /// 3270_EraseInput
    pub const KEY_3270_EraseInput: Keysym = Keysym(0x0000fd07);
    /// 3270_Reset
    pub const KEY_3270_Reset: Keysym = Keysym(0x0000fd08);
    /// 3270_Quit
    pub const KEY_3270_Quit: Keysym = Keysym(0x0000fd09);
    /// 3270_PA1
    pub const KEY_3270_PA1: Keysym = Keysym(0x0000fd0a);
    /// 3270_PA2
    pub const KEY_3270_PA2: Keysym = Keysym(0x0000fd0b);
    /// 3270_PA3
    pub const KEY_3270_PA3: Keysym = Keysym(0x0000fd0c);
    /// 3270_Test
    pub const KEY_3270_Test: Keysym = Keysym(0x0000fd0d);
    /// 3270_Attn
    pub const KEY_3270_Attn: Keysym = Keysym(0x0000fd0e);
    /// 3270_CursorBlink
    pub const KEY_3270_CursorBlink: Keysym = Keysym(0x0000fd0f);
    /// 3270_AltCursor
    pub const KEY_3270_AltCursor: Keysym = Keysym(0x0000fd10);
    /// 3270_KeyClick
    pub const KEY_3270_KeyClick: Keysym = Keysym(0x0000fd11);
    /// 3270_Jump
    pub const KEY_3270_Jump: Keysym = Keysym(0x0000fd12);
    /// 3270_Ident
    pub const KEY_3270_Ident: Keysym = Keysym(0x0000fd13);
    /// 3270_Rule
    pub const KEY_3270_Rule: Keysym = Keysym(0x0000fd14);
    /// 3270_Copy
    pub const KEY_3270_Copy: Keysym = Keysym(0x0000fd15);
    /// 3270_Play
    pub const KEY_3270_Play: Keysym = Keysym(0x0000fd16);
    /// 3270_Setup
    pub const KEY_3270_Setup: Keysym = Keysym(0x0000fd17);
    /// 3270_Record
    pub const KEY_3270_Record: Keysym = Keysym(0x0000fd18);
    /// 3270_ChangeScreen
    pub const KEY_3270_ChangeScreen: Keysym = Keysym(0x0000fd19);
    /// 3270_DeleteWord
    pub const KEY_3270_DeleteWord: Keysym = Keysym(0x0000fd1a);
    /// 3270_ExSelect
    pub const KEY_3270_ExSelect: Keysym = Keysym(0x0000fd1b);
    /// 3270_CursorSelect
    pub const KEY_3270_CursorSelect: Keysym = Keysym(0x0000fd1c);
    /// 3270_PrintScreen
    pub const KEY_3270_PrintScreen: Keysym = Keysym(0x0000fd1d);
    /// 3270_Enter
    pub const KEY_3270_Enter: Keysym = Keysym(0x0000fd1e);
    /// space
    pub const KEY_space: Keysym = Keysym(0x00000020);
    /// exclam
    pub const KEY_exclam: Keysym = Keysym(0x00000021);
    /// quotedbl
    pub const KEY_quotedbl: Keysym = Keysym(0x00000022);
    /// numbersign
    pub const KEY_numbersign: Keysym = Keysym(0x00000023);
    /// dollar
    pub const KEY_dollar: Keysym = Keysym(0x00000024);
    /// percent
    pub const KEY_percent: Keysym = Keysym(0x00000025);
    /// ampersand
    pub const KEY_ampersand: Keysym = Keysym(0x00000026);
    /// apostrophe
    pub const KEY_apostrophe: Keysym = Keysym(0x00000027);
    /// quoteright
    pub const KEY_quoteright: Keysym = Keysym(0x00000027);
    /// parenleft
    pub const KEY_parenleft: Keysym = Keysym(0x00000028);
    /// parenright
    pub const KEY_parenright: Keysym = Keysym(0x00000029);
    /// asterisk
    pub const KEY_asterisk: Keysym = Keysym(0x0000002a);
    /// plus
    pub const KEY_plus: Keysym = Keysym(0x0000002b);
    /// comma
    pub const KEY_comma: Keysym = Keysym(0x0000002c);
    /// minus
    pub const KEY_minus: Keysym = Keysym(0x0000002d);
    /// period
    pub const KEY_period: Keysym = Keysym(0x0000002e);
    /// slash
    pub const KEY_slash: Keysym = Keysym(0x0000002f);
    /// 0
    pub const KEY_0: Keysym = Keysym(0x00000030);
    /// 1
    pub const KEY_1: Keysym = Keysym(0x00000031);
    /// 2
    pub const KEY_2: Keysym = Keysym(0x00000032);
    /// 3
    pub const KEY_3: Keysym = Keysym(0x00000033);
    /// 4
    pub const KEY_4: Keysym = Keysym(0x00000034);
    /// 5
    pub const KEY_5: Keysym = Keysym(0x00000035);
    /// 6
    pub const KEY_6: Keysym = Keysym(0x00000036);
    /// 7
    pub const KEY_7: Keysym = Keysym(0x00000037);
    /// 8
    pub const KEY_8: Keysym = Keysym(0x00000038);
    /// 9
    pub const KEY_9: Keysym = Keysym(0x00000039);
    /// colon
    pub const KEY_colon: Keysym = Keysym(0x0000003a);
    /// semicolon
    pub const KEY_semicolon: Keysym = Keysym(0x0000003b);
    /// less
    pub const KEY_less: Keysym = Keysym(0x0000003c);
    /// equal
    pub const KEY_equal: Keysym = Keysym(0x0000003d);
    /// greater
    pub const KEY_greater: Keysym = Keysym(0x0000003e);
    /// question
    pub const KEY_question: Keysym = Keysym(0x0000003f);
    /// at
    pub const KEY_at: Keysym = Keysym(0x00000040);
    /// A
    pub const KEY_A: Keysym = Keysym(0x00000041);
    /// B
    pub const KEY_B: Keysym = Keysym(0x00000042);
    /// C
    pub const KEY_C: Keysym = Keysym(0x00000043);
    /// D
    pub const KEY_D: Keysym = Keysym(0x00000044);
    /// E
    pub const KEY_E: Keysym = Keysym(0x00000045);
    /// F
    pub const KEY_F: Keysym = Keysym(0x00000046);
    /// G
    pub const KEY_G: Keysym = Keysym(0x00000047);
    /// H
    pub const KEY_H: Keysym = Keysym(0x00000048);
    /// I
    pub const KEY_I: Keysym = Keysym(0x00000049);
    /// J
    pub const KEY_J: Keysym = Keysym(0x0000004a);
    /// K
    pub const KEY_K: Keysym = Keysym(0x0000004b);
    /// L
    pub const KEY_L: Keysym = Keysym(0x0000004c);
    /// M
    pub const KEY_M: Keysym = Keysym(0x0000004d);
    /// N
    pub const KEY_N: Keysym = Keysym(0x0000004e);
    /// O
    pub const KEY_O: Keysym = Keysym(0x0000004f);
    /// P
    pub const KEY_P: Keysym = Keysym(0x00000050);
    /// Q
    pub const KEY_Q: Keysym = Keysym(0x00000051);
    /// R
    pub const KEY_R: Keysym = Keysym(0x00000052);
    /// S
    pub const KEY_S: Keysym = Keysym(0x00000053);
    /// T
    pub const KEY_T: Keysym = Keysym(0x00000054);
    /// U
    pub const KEY_U: Keysym = Keysym(0x00000055);
    /// V
    pub const KEY_V: Keysym = Keysym(0x00000056);
    /// W
    pub const KEY_W: Keysym = Keysym(0x00000057);
    /// X
    pub const KEY_X: Keysym = Keysym(0x00000058);
    /// Y
    pub const KEY_Y: Keysym = Keysym(0x00000059);
    /// Z
    pub const KEY_Z: Keysym = Keysym(0x0000005a);
    /// bracketleft
    pub const KEY_bracketleft: Keysym = Keysym(0x0000005b);
    /// backslash
    pub const KEY_backslash: Keysym = Keysym(0x0000005c);
    /// bracketright
    pub const KEY_bracketright: Keysym = Keysym(0x0000005d);
    /// asciicircum
    pub const KEY_asciicircum: Keysym = Keysym(0x0000005e);
    /// underscore
    pub const KEY_underscore: Keysym = Keysym(0x0000005f);
    /// grave
    pub const KEY_grave: Keysym = Keysym(0x00000060);
    /// quoteleft
    pub const KEY_quoteleft: Keysym = Keysym(0x00000060);
    /// a
    pub const KEY_a: Keysym = Keysym(0x00000061);
    /// b
    pub const KEY_b: Keysym = Keysym(0x00000062);
    /// c
    pub const KEY_c: Keysym = Keysym(0x00000063);
    /// d
    pub const KEY_d: Keysym = Keysym(0x00000064);
    /// e
    pub const KEY_e: Keysym = Keysym(0x00000065);
    /// f
    pub const KEY_f: Keysym = Keysym(0x00000066);
    /// g
    pub const KEY_g: Keysym = Keysym(0x00000067);
    /// h
    pub const KEY_h: Keysym = Keysym(0x00000068);
    /// i
    pub const KEY_i: Keysym = Keysym(0x00000069);
    /// j
    pub const KEY_j: Keysym = Keysym(0x0000006a);
    /// k
    pub const KEY_k: Keysym = Keysym(0x0000006b);
    /// l
    pub const KEY_l: Keysym = Keysym(0x0000006c);
    /// m
    pub const KEY_m: Keysym = Keysym(0x0000006d);
    /// n
    pub const KEY_n: Keysym = Keysym(0x0000006e);
    /// o
    pub const KEY_o: Keysym = Keysym(0x0000006f);
    /// p
    pub const KEY_p: Keysym = Keysym(0x00000070);
    /// q
    pub const KEY_q: Keysym = Keysym(0x00000071);
    /// r
    pub const KEY_r: Keysym = Keysym(0x00000072);
    /// s
    pub const KEY_s: Keysym = Keysym(0x00000073);
    /// t
    pub const KEY_t: Keysym = Keysym(0x00000074);
    /// u
    pub const KEY_u: Keysym = Keysym(0x00000075);
    /// v
    pub const KEY_v: Keysym = Keysym(0x00000076);
    /// w
    pub const KEY_w: Keysym = Keysym(0x00000077);
    /// x
    pub const KEY_x: Keysym = Keysym(0x00000078);
    /// y
    pub const KEY_y: Keysym = Keysym(0x00000079);
    /// z
    pub const KEY_z: Keysym = Keysym(0x0000007a);
    /// braceleft
    pub const KEY_braceleft: Keysym = Keysym(0x0000007b);
    /// bar
    pub const KEY_bar: Keysym = Keysym(0x0000007c);
    /// braceright
    pub const KEY_braceright: Keysym = Keysym(0x0000007d);
    /// asciitilde
    pub const KEY_asciitilde: Keysym = Keysym(0x0000007e);
    /// nobreakspace
    pub const KEY_nobreakspace: Keysym = Keysym(0x000000a0);
    /// exclamdown
    pub const KEY_exclamdown: Keysym = Keysym(0x000000a1);
    /// cent
    pub const KEY_cent: Keysym = Keysym(0x000000a2);
    /// sterling
    pub const KEY_sterling: Keysym = Keysym(0x000000a3);
    /// currency
    pub const KEY_currency: Keysym = Keysym(0x000000a4);
    /// yen
    pub const KEY_yen: Keysym = Keysym(0x000000a5);
    /// brokenbar
    pub const KEY_brokenbar: Keysym = Keysym(0x000000a6);
    /// section
    pub const KEY_section: Keysym = Keysym(0x000000a7);
    /// diaeresis
    pub const KEY_diaeresis: Keysym = Keysym(0x000000a8);
    /// copyright
    pub const KEY_copyright: Keysym = Keysym(0x000000a9);
    /// ordfeminine
    pub const KEY_ordfeminine: Keysym = Keysym(0x000000aa);
    /// guillemotleft
    pub const KEY_guillemotleft: Keysym = Keysym(0x000000ab);
    /// guillemetleft
    pub const KEY_guillemetleft: Keysym = Keysym(0x000000ab);
    /// notsign
    pub const KEY_notsign: Keysym = Keysym(0x000000ac);
    /// hyphen
    pub const KEY_hyphen: Keysym = Keysym(0x000000ad);
    /// registered
    pub const KEY_registered: Keysym = Keysym(0x000000ae);
    /// macron
    pub const KEY_macron: Keysym = Keysym(0x000000af);
    /// degree
    pub const KEY_degree: Keysym = Keysym(0x000000b0);
    /// plusminus
    pub const KEY_plusminus: Keysym = Keysym(0x000000b1);
    /// twosuperior
    pub const KEY_twosuperior: Keysym = Keysym(0x000000b2);
    /// threesuperior
    pub const KEY_threesuperior: Keysym = Keysym(0x000000b3);
    /// acute
    pub const KEY_acute: Keysym = Keysym(0x000000b4);
    /// mu
    pub const KEY_mu: Keysym = Keysym(0x000000b5);
    /// paragraph
    pub const KEY_paragraph: Keysym = Keysym(0x000000b6);
    /// periodcentered
    pub const KEY_periodcentered: Keysym = Keysym(0x000000b7);
    /// cedilla
    pub const KEY_cedilla: Keysym = Keysym(0x000000b8);
    /// onesuperior
    pub const KEY_onesuperior: Keysym = Keysym(0x000000b9);
    /// masculine
    pub const KEY_masculine: Keysym = Keysym(0x000000ba);
    /// ordmasculine
    pub const KEY_ordmasculine: Keysym = Keysym(0x000000ba);
    /// guillemotright
    pub const KEY_guillemotright: Keysym = Keysym(0x000000bb);
    /// guillemetright
    pub const KEY_guillemetright: Keysym = Keysym(0x000000bb);
    /// onequarter
    pub const KEY_onequarter: Keysym = Keysym(0x000000bc);
    /// onehalf
    pub const KEY_onehalf: Keysym = Keysym(0x000000bd);
    /// threequarters
    pub const KEY_threequarters: Keysym = Keysym(0x000000be);
    /// questiondown
    pub const KEY_questiondown: Keysym = Keysym(0x000000bf);
    /// Agrave
    pub const KEY_Agrave: Keysym = Keysym(0x000000c0);
    /// Aacute
    pub const KEY_Aacute: Keysym = Keysym(0x000000c1);
    /// Acircumflex
    pub const KEY_Acircumflex: Keysym = Keysym(0x000000c2);
    /// Atilde
    pub const KEY_Atilde: Keysym = Keysym(0x000000c3);
    /// Adiaeresis
    pub const KEY_Adiaeresis: Keysym = Keysym(0x000000c4);
    /// Aring
    pub const KEY_Aring: Keysym = Keysym(0x000000c5);
    /// AE
    pub const KEY_AE: Keysym = Keysym(0x000000c6);
    /// Ccedilla
    pub const KEY_Ccedilla: Keysym = Keysym(0x000000c7);
    /// Egrave
    pub const KEY_Egrave: Keysym = Keysym(0x000000c8);
    /// Eacute
    pub const KEY_Eacute: Keysym = Keysym(0x000000c9);
    /// Ecircumflex
    pub const KEY_Ecircumflex: Keysym = Keysym(0x000000ca);
    /// Ediaeresis
    pub const KEY_Ediaeresis: Keysym = Keysym(0x000000cb);
    /// Igrave
    pub const KEY_Igrave: Keysym = Keysym(0x000000cc);
    /// Iacute
    pub const KEY_Iacute: Keysym = Keysym(0x000000cd);
    /// Icircumflex
    pub const KEY_Icircumflex: Keysym = Keysym(0x000000ce);
    /// Idiaeresis
    pub const KEY_Idiaeresis: Keysym = Keysym(0x000000cf);
    /// ETH
    pub const KEY_ETH: Keysym = Keysym(0x000000d0);
    /// Eth
    pub const KEY_Eth: Keysym = Keysym(0x000000d0);
    /// Ntilde
    pub const KEY_Ntilde: Keysym = Keysym(0x000000d1);
    /// Ograve
    pub const KEY_Ograve: Keysym = Keysym(0x000000d2);
    /// Oacute
    pub const KEY_Oacute: Keysym = Keysym(0x000000d3);
    /// Ocircumflex
    pub const KEY_Ocircumflex: Keysym = Keysym(0x000000d4);
    /// Otilde
    pub const KEY_Otilde: Keysym = Keysym(0x000000d5);
    /// Odiaeresis
    pub const KEY_Odiaeresis: Keysym = Keysym(0x000000d6);
    /// multiply
    pub const KEY_multiply: Keysym = Keysym(0x000000d7);
    /// Oslash
    pub const KEY_Oslash: Keysym = Keysym(0x000000d8);
    /// Ooblique
    pub const KEY_Ooblique: Keysym = Keysym(0x000000d8);
    /// Ugrave
    pub const KEY_Ugrave: Keysym = Keysym(0x000000d9);
    /// Uacute
    pub const KEY_Uacute: Keysym = Keysym(0x000000da);
    /// Ucircumflex
    pub const KEY_Ucircumflex: Keysym = Keysym(0x000000db);
    /// Udiaeresis
    pub const KEY_Udiaeresis: Keysym = Keysym(0x000000dc);
    /// Yacute
    pub const KEY_Yacute: Keysym = Keysym(0x000000dd);
    /// THORN
    pub const KEY_THORN: Keysym = Keysym(0x000000de);
    /// Thorn
    pub const KEY_Thorn: Keysym = Keysym(0x000000de);
    /// ssharp
    pub const KEY_ssharp: Keysym = Keysym(0x000000df);
    /// agrave
    pub const KEY_agrave: Keysym = Keysym(0x000000e0);
    /// aacute
    pub const KEY_aacute: Keysym = Keysym(0x000000e1);
    /// acircumflex
    pub const KEY_acircumflex: Keysym = Keysym(0x000000e2);
    /// atilde
    pub const KEY_atilde: Keysym = Keysym(0x000000e3);
    /// adiaeresis
    pub const KEY_adiaeresis: Keysym = Keysym(0x000000e4);
    /// aring
    pub const KEY_aring: Keysym = Keysym(0x000000e5);
    /// ae
    pub const KEY_ae: Keysym = Keysym(0x000000e6);
    /// ccedilla
    pub const KEY_ccedilla: Keysym = Keysym(0x000000e7);
    /// egrave
    pub const KEY_egrave: Keysym = Keysym(0x000000e8);
    /// eacute
    pub const KEY_eacute: Keysym = Keysym(0x000000e9);
    /// ecircumflex
    pub const KEY_ecircumflex: Keysym = Keysym(0x000000ea);
    /// ediaeresis
    pub const KEY_ediaeresis: Keysym = Keysym(0x000000eb);
    /// igrave
    pub const KEY_igrave: Keysym = Keysym(0x000000ec);
    /// iacute
    pub const KEY_iacute: Keysym = Keysym(0x000000ed);
    /// icircumflex
    pub const KEY_icircumflex: Keysym = Keysym(0x000000ee);
    /// idiaeresis
    pub const KEY_idiaeresis: Keysym = Keysym(0x000000ef);
    /// eth
    pub const KEY_eth: Keysym = Keysym(0x000000f0);
    /// ntilde
    pub const KEY_ntilde: Keysym = Keysym(0x000000f1);
    /// ograve
    pub const KEY_ograve: Keysym = Keysym(0x000000f2);
    /// oacute
    pub const KEY_oacute: Keysym = Keysym(0x000000f3);
    /// ocircumflex
    pub const KEY_ocircumflex: Keysym = Keysym(0x000000f4);
    /// otilde
    pub const KEY_otilde: Keysym = Keysym(0x000000f5);
    /// odiaeresis
    pub const KEY_odiaeresis: Keysym = Keysym(0x000000f6);
    /// division
    pub const KEY_division: Keysym = Keysym(0x000000f7);
    /// oslash
    pub const KEY_oslash: Keysym = Keysym(0x000000f8);
    /// ooblique
    pub const KEY_ooblique: Keysym = Keysym(0x000000f8);
    /// ugrave
    pub const KEY_ugrave: Keysym = Keysym(0x000000f9);
    /// uacute
    pub const KEY_uacute: Keysym = Keysym(0x000000fa);
    /// ucircumflex
    pub const KEY_ucircumflex: Keysym = Keysym(0x000000fb);
    /// udiaeresis
    pub const KEY_udiaeresis: Keysym = Keysym(0x000000fc);
    /// yacute
    pub const KEY_yacute: Keysym = Keysym(0x000000fd);
    /// thorn
    pub const KEY_thorn: Keysym = Keysym(0x000000fe);
    /// ydiaeresis
    pub const KEY_ydiaeresis: Keysym = Keysym(0x000000ff);
    /// Aogonek
    pub const KEY_Aogonek: Keysym = Keysym(0x000001a1);
    /// breve
    pub const KEY_breve: Keysym = Keysym(0x000001a2);
    /// Lstroke
    pub const KEY_Lstroke: Keysym = Keysym(0x000001a3);
    /// Lcaron
    pub const KEY_Lcaron: Keysym = Keysym(0x000001a5);
    /// Sacute
    pub const KEY_Sacute: Keysym = Keysym(0x000001a6);
    /// Scaron
    pub const KEY_Scaron: Keysym = Keysym(0x000001a9);
    /// Scedilla
    pub const KEY_Scedilla: Keysym = Keysym(0x000001aa);
    /// Tcaron
    pub const KEY_Tcaron: Keysym = Keysym(0x000001ab);
    /// Zacute
    pub const KEY_Zacute: Keysym = Keysym(0x000001ac);
    /// Zcaron
    pub const KEY_Zcaron: Keysym = Keysym(0x000001ae);
    /// Zabovedot
    pub const KEY_Zabovedot: Keysym = Keysym(0x000001af);
    /// aogonek
    pub const KEY_aogonek: Keysym = Keysym(0x000001b1);
    /// ogonek
    pub const KEY_ogonek: Keysym = Keysym(0x000001b2);
    /// lstroke
    pub const KEY_lstroke: Keysym = Keysym(0x000001b3);
    /// lcaron
    pub const KEY_lcaron: Keysym = Keysym(0x000001b5);
    /// sacute
    pub const KEY_sacute: Keysym = Keysym(0x000001b6);
    /// caron
    pub const KEY_caron: Keysym = Keysym(0x000001b7);
    /// scaron
    pub const KEY_scaron: Keysym = Keysym(0x000001b9);
    /// scedilla
    pub const KEY_scedilla: Keysym = Keysym(0x000001ba);
    /// tcaron
    pub const KEY_tcaron: Keysym = Keysym(0x000001bb);
    /// zacute
    pub const KEY_zacute: Keysym = Keysym(0x000001bc);
    /// doubleacute
    pub const KEY_doubleacute: Keysym = Keysym(0x000001bd);
    /// zcaron
    pub const KEY_zcaron: Keysym = Keysym(0x000001be);
    /// zabovedot
    pub const KEY_zabovedot: Keysym = Keysym(0x000001bf);
    /// Racute
    pub const KEY_Racute: Keysym = Keysym(0x000001c0);
    /// Abreve
    pub const KEY_Abreve: Keysym = Keysym(0x000001c3);
    /// Lacute
    pub const KEY_Lacute: Keysym = Keysym(0x000001c5);
    /// Cacute
    pub const KEY_Cacute: Keysym = Keysym(0x000001c6);
    /// Ccaron
    pub const KEY_Ccaron: Keysym = Keysym(0x000001c8);
    /// Eogonek
    pub const KEY_Eogonek: Keysym = Keysym(0x000001ca);
    /// Ecaron
    pub const KEY_Ecaron: Keysym = Keysym(0x000001cc);
    /// Dcaron
    pub const KEY_Dcaron: Keysym = Keysym(0x000001cf);
    /// Dstroke
    pub const KEY_Dstroke: Keysym = Keysym(0x000001d0);
    /// Nacute
    pub const KEY_Nacute: Keysym = Keysym(0x000001d1);
    /// Ncaron
    pub const KEY_Ncaron: Keysym = Keysym(0x000001d2);
    /// Odoubleacute
    pub const KEY_Odoubleacute: Keysym = Keysym(0x000001d5);
    /// Rcaron
    pub const KEY_Rcaron: Keysym = Keysym(0x000001d8);
    /// Uring
    pub const KEY_Uring: Keysym = Keysym(0x000001d9);
    /// Udoubleacute
    pub const KEY_Udoubleacute: Keysym = Keysym(0x000001db);
    /// Tcedilla
    pub const KEY_Tcedilla: Keysym = Keysym(0x000001de);
    /// racute
    pub const KEY_racute: Keysym = Keysym(0x000001e0);
    /// abreve
    pub const KEY_abreve: Keysym = Keysym(0x000001e3);
    /// lacute
    pub const KEY_lacute: Keysym = Keysym(0x000001e5);
    /// cacute
    pub const KEY_cacute: Keysym = Keysym(0x000001e6);
    /// ccaron
    pub const KEY_ccaron: Keysym = Keysym(0x000001e8);
    /// eogonek
    pub const KEY_eogonek: Keysym = Keysym(0x000001ea);
    /// ecaron
    pub const KEY_ecaron: Keysym = Keysym(0x000001ec);
    /// dcaron
    pub const KEY_dcaron: Keysym = Keysym(0x000001ef);
    /// dstroke
    pub const KEY_dstroke: Keysym = Keysym(0x000001f0);
    /// nacute
    pub const KEY_nacute: Keysym = Keysym(0x000001f1);
    /// ncaron
    pub const KEY_ncaron: Keysym = Keysym(0x000001f2);
    /// odoubleacute
    pub const KEY_odoubleacute: Keysym = Keysym(0x000001f5);
    /// rcaron
    pub const KEY_rcaron: Keysym = Keysym(0x000001f8);
    /// uring
    pub const KEY_uring: Keysym = Keysym(0x000001f9);
    /// udoubleacute
    pub const KEY_udoubleacute: Keysym = Keysym(0x000001fb);
    /// tcedilla
    pub const KEY_tcedilla: Keysym = Keysym(0x000001fe);
    /// abovedot
    pub const KEY_abovedot: Keysym = Keysym(0x000001ff);
    /// Hstroke
    pub const KEY_Hstroke: Keysym = Keysym(0x000002a1);
    /// Hcircumflex
    pub const KEY_Hcircumflex: Keysym = Keysym(0x000002a6);
    /// Iabovedot
    pub const KEY_Iabovedot: Keysym = Keysym(0x000002a9);
    /// Gbreve
    pub const KEY_Gbreve: Keysym = Keysym(0x000002ab);
    /// Jcircumflex
    pub const KEY_Jcircumflex: Keysym = Keysym(0x000002ac);
    /// hstroke
    pub const KEY_hstroke: Keysym = Keysym(0x000002b1);
    /// hcircumflex
    pub const KEY_hcircumflex: Keysym = Keysym(0x000002b6);
    /// idotless
    pub const KEY_idotless: Keysym = Keysym(0x000002b9);
    /// gbreve
    pub const KEY_gbreve: Keysym = Keysym(0x000002bb);
    /// jcircumflex
    pub const KEY_jcircumflex: Keysym = Keysym(0x000002bc);
    /// Cabovedot
    pub const KEY_Cabovedot: Keysym = Keysym(0x000002c5);
    /// Ccircumflex
    pub const KEY_Ccircumflex: Keysym = Keysym(0x000002c6);
    /// Gabovedot
    pub const KEY_Gabovedot: Keysym = Keysym(0x000002d5);
    /// Gcircumflex
    pub const KEY_Gcircumflex: Keysym = Keysym(0x000002d8);
    /// Ubreve
    pub const KEY_Ubreve: Keysym = Keysym(0x000002dd);
    /// Scircumflex
    pub const KEY_Scircumflex: Keysym = Keysym(0x000002de);
    /// cabovedot
    pub const KEY_cabovedot: Keysym = Keysym(0x000002e5);
    /// ccircumflex
    pub const KEY_ccircumflex: Keysym = Keysym(0x000002e6);
    /// gabovedot
    pub const KEY_gabovedot: Keysym = Keysym(0x000002f5);
    /// gcircumflex
    pub const KEY_gcircumflex: Keysym = Keysym(0x000002f8);
    /// ubreve
    pub const KEY_ubreve: Keysym = Keysym(0x000002fd);
    /// scircumflex
    pub const KEY_scircumflex: Keysym = Keysym(0x000002fe);
    /// kra
    pub const KEY_kra: Keysym = Keysym(0x000003a2);
    /// kappa
    pub const KEY_kappa: Keysym = Keysym(0x000003a2);
    /// Rcedilla
    pub const KEY_Rcedilla: Keysym = Keysym(0x000003a3);
    /// Itilde
    pub const KEY_Itilde: Keysym = Keysym(0x000003a5);
    /// Lcedilla
    pub const KEY_Lcedilla: Keysym = Keysym(0x000003a6);
    /// Emacron
    pub const KEY_Emacron: Keysym = Keysym(0x000003aa);
    /// Gcedilla
    pub const KEY_Gcedilla: Keysym = Keysym(0x000003ab);
    /// Tslash
    pub const KEY_Tslash: Keysym = Keysym(0x000003ac);
    /// rcedilla
    pub const KEY_rcedilla: Keysym = Keysym(0x000003b3);
    /// itilde
    pub const KEY_itilde: Keysym = Keysym(0x000003b5);
    /// lcedilla
    pub const KEY_lcedilla: Keysym = Keysym(0x000003b6);
    /// emacron
    pub const KEY_emacron: Keysym = Keysym(0x000003ba);
    /// gcedilla
    pub const KEY_gcedilla: Keysym = Keysym(0x000003bb);
    /// tslash
    pub const KEY_tslash: Keysym = Keysym(0x000003bc);
    /// ENG
    pub const KEY_ENG: Keysym = Keysym(0x000003bd);
    /// eng
    pub const KEY_eng: Keysym = Keysym(0x000003bf);
    /// Amacron
    pub const KEY_Amacron: Keysym = Keysym(0x000003c0);
    /// Iogonek
    pub const KEY_Iogonek: Keysym = Keysym(0x000003c7);
    /// Eabovedot
    pub const KEY_Eabovedot: Keysym = Keysym(0x000003cc);
    /// Imacron
    pub const KEY_Imacron: Keysym = Keysym(0x000003cf);
    /// Ncedilla
    pub const KEY_Ncedilla: Keysym = Keysym(0x000003d1);
    /// Omacron
    pub const KEY_Omacron: Keysym = Keysym(0x000003d2);
    /// Kcedilla
    pub const KEY_Kcedilla: Keysym = Keysym(0x000003d3);
    /// Uogonek
    pub const KEY_Uogonek: Keysym = Keysym(0x000003d9);
    /// Utilde
    pub const KEY_Utilde: Keysym = Keysym(0x000003dd);
    /// Umacron
    pub const KEY_Umacron: Keysym = Keysym(0x000003de);
    /// amacron
    pub const KEY_amacron: Keysym = Keysym(0x000003e0);
    /// iogonek
    pub const KEY_iogonek: Keysym = Keysym(0x000003e7);
    /// eabovedot
    pub const KEY_eabovedot: Keysym = Keysym(0x000003ec);
    /// imacron
    pub const KEY_imacron: Keysym = Keysym(0x000003ef);
    /// ncedilla
    pub const KEY_ncedilla: Keysym = Keysym(0x000003f1);
    /// omacron
    pub const KEY_omacron: Keysym = Keysym(0x000003f2);
    /// kcedilla
    pub const KEY_kcedilla: Keysym = Keysym(0x000003f3);
    /// uogonek
    pub const KEY_uogonek: Keysym = Keysym(0x000003f9);
    /// utilde
    pub const KEY_utilde: Keysym = Keysym(0x000003fd);
    /// umacron
    pub const KEY_umacron: Keysym = Keysym(0x000003fe);
    /// Wcircumflex
    pub const KEY_Wcircumflex: Keysym = Keysym(0x01000174);
    /// wcircumflex
    pub const KEY_wcircumflex: Keysym = Keysym(0x01000175);
    /// Ycircumflex
    pub const KEY_Ycircumflex: Keysym = Keysym(0x01000176);
    /// ycircumflex
    pub const KEY_ycircumflex: Keysym = Keysym(0x01000177);
    /// Babovedot
    pub const KEY_Babovedot: Keysym = Keysym(0x01001e02);
    /// babovedot
    pub const KEY_babovedot: Keysym = Keysym(0x01001e03);
    /// Dabovedot
    pub const KEY_Dabovedot: Keysym = Keysym(0x01001e0a);
    /// dabovedot
    pub const KEY_dabovedot: Keysym = Keysym(0x01001e0b);
    /// Fabovedot
    pub const KEY_Fabovedot: Keysym = Keysym(0x01001e1e);
    /// fabovedot
    pub const KEY_fabovedot: Keysym = Keysym(0x01001e1f);
    /// Mabovedot
    pub const KEY_Mabovedot: Keysym = Keysym(0x01001e40);
    /// mabovedot
    pub const KEY_mabovedot: Keysym = Keysym(0x01001e41);
    /// Pabovedot
    pub const KEY_Pabovedot: Keysym = Keysym(0x01001e56);
    /// pabovedot
    pub const KEY_pabovedot: Keysym = Keysym(0x01001e57);
    /// Sabovedot
    pub const KEY_Sabovedot: Keysym = Keysym(0x01001e60);
    /// sabovedot
    pub const KEY_sabovedot: Keysym = Keysym(0x01001e61);
    /// Tabovedot
    pub const KEY_Tabovedot: Keysym = Keysym(0x01001e6a);
    /// tabovedot
    pub const KEY_tabovedot: Keysym = Keysym(0x01001e6b);
    /// Wgrave
    pub const KEY_Wgrave: Keysym = Keysym(0x01001e80);
    /// wgrave
    pub const KEY_wgrave: Keysym = Keysym(0x01001e81);
    /// Wacute
    pub const KEY_Wacute: Keysym = Keysym(0x01001e82);
    /// wacute
    pub const KEY_wacute: Keysym = Keysym(0x01001e83);
    /// Wdiaeresis
    pub const KEY_Wdiaeresis: Keysym = Keysym(0x01001e84);
    /// wdiaeresis
    pub const KEY_wdiaeresis: Keysym = Keysym(0x01001e85);
    /// Ygrave
    pub const KEY_Ygrave: Keysym = Keysym(0x01001ef2);
    /// ygrave
    pub const KEY_ygrave: Keysym = Keysym(0x01001ef3);
    /// OE
    pub const KEY_OE: Keysym = Keysym(0x000013bc);
    /// oe
    pub const KEY_oe: Keysym = Keysym(0x000013bd);
    /// Ydiaeresis
    pub const KEY_Ydiaeresis: Keysym = Keysym(0x000013be);
    /// overline
    pub const KEY_overline: Keysym = Keysym(0x0000047e);
    /// kana_fullstop
    pub const KEY_kana_fullstop: Keysym = Keysym(0x000004a1);
    /// kana_openingbracket
    pub const KEY_kana_openingbracket: Keysym = Keysym(0x000004a2);
    /// kana_closingbracket
    pub const KEY_kana_closingbracket: Keysym = Keysym(0x000004a3);
    /// kana_comma
    pub const KEY_kana_comma: Keysym = Keysym(0x000004a4);
    /// kana_conjunctive
    pub const KEY_kana_conjunctive: Keysym = Keysym(0x000004a5);
    /// kana_middledot
    pub const KEY_kana_middledot: Keysym = Keysym(0x000004a5);
    /// kana_WO
    pub const KEY_kana_WO: Keysym = Keysym(0x000004a6);
    /// kana_a
    pub const KEY_kana_a: Keysym = Keysym(0x000004a7);
    /// kana_i
    pub const KEY_kana_i: Keysym = Keysym(0x000004a8);
    /// kana_u
    pub const KEY_kana_u: Keysym = Keysym(0x000004a9);
    /// kana_e
    pub const KEY_kana_e: Keysym = Keysym(0x000004aa);
    /// kana_o
    pub const KEY_kana_o: Keysym = Keysym(0x000004ab);
    /// kana_ya
    pub const KEY_kana_ya: Keysym = Keysym(0x000004ac);
    /// kana_yu
    pub const KEY_kana_yu: Keysym = Keysym(0x000004ad);
    /// kana_yo
    pub const KEY_kana_yo: Keysym = Keysym(0x000004ae);
    /// kana_tsu
    pub const KEY_kana_tsu: Keysym = Keysym(0x000004af);
    /// kana_tu
    pub const KEY_kana_tu: Keysym = Keysym(0x000004af);
    /// prolongedsound
    pub const KEY_prolongedsound: Keysym = Keysym(0x000004b0);
    /// kana_A
    pub const KEY_kana_A: Keysym = Keysym(0x000004b1);
    /// kana_I
    pub const KEY_kana_I: Keysym = Keysym(0x000004b2);
    /// kana_U
    pub const KEY_kana_U: Keysym = Keysym(0x000004b3);
    /// kana_E
    pub const KEY_kana_E: Keysym = Keysym(0x000004b4);
    /// kana_O
    pub const KEY_kana_O: Keysym = Keysym(0x000004b5);
    /// kana_KA
    pub const KEY_kana_KA: Keysym = Keysym(0x000004b6);
    /// kana_KI
    pub const KEY_kana_KI: Keysym = Keysym(0x000004b7);
    /// kana_KU
    pub const KEY_kana_KU: Keysym = Keysym(0x000004b8);
    /// kana_KE
    pub const KEY_kana_KE: Keysym = Keysym(0x000004b9);
    /// kana_KO
    pub const KEY_kana_KO: Keysym = Keysym(0x000004ba);
    /// kana_SA
    pub const KEY_kana_SA: Keysym = Keysym(0x000004bb);
    /// kana_SHI
    pub const KEY_kana_SHI: Keysym = Keysym(0x000004bc);
    /// kana_SU
    pub const KEY_kana_SU: Keysym = Keysym(0x000004bd);
    /// kana_SE
    pub const KEY_kana_SE: Keysym = Keysym(0x000004be);
    /// kana_SO
    pub const KEY_kana_SO: Keysym = Keysym(0x000004bf);
    /// kana_TA
    pub const KEY_kana_TA: Keysym = Keysym(0x000004c0);
    /// kana_CHI
    pub const KEY_kana_CHI: Keysym = Keysym(0x000004c1);
    /// kana_TI
    pub const KEY_kana_TI: Keysym = Keysym(0x000004c1);
    /// kana_TSU
    pub const KEY_kana_TSU: Keysym = Keysym(0x000004c2);
    /// kana_TU
    pub const KEY_kana_TU: Keysym = Keysym(0x000004c2);
    /// kana_TE
    pub const KEY_kana_TE: Keysym = Keysym(0x000004c3);
    /// kana_TO
    pub const KEY_kana_TO: Keysym = Keysym(0x000004c4);
    /// kana_NA
    pub const KEY_kana_NA: Keysym = Keysym(0x000004c5);
    /// kana_NI
    pub const KEY_kana_NI: Keysym = Keysym(0x000004c6);
    /// kana_NU
    pub const KEY_kana_NU: Keysym = Keysym(0x000004c7);
    /// kana_NE
    pub const KEY_kana_NE: Keysym = Keysym(0x000004c8);
    /// kana_NO
    pub const KEY_kana_NO: Keysym = Keysym(0x000004c9);
    /// kana_HA
    pub const KEY_kana_HA: Keysym = Keysym(0x000004ca);
    /// kana_HI
    pub const KEY_kana_HI: Keysym = Keysym(0x000004cb);
    /// kana_FU
    pub const KEY_kana_FU: Keysym = Keysym(0x000004cc);
    /// kana_HU
    pub const KEY_kana_HU: Keysym = Keysym(0x000004cc);
    /// kana_HE
    pub const KEY_kana_HE: Keysym = Keysym(0x000004cd);
    /// kana_HO
    pub const KEY_kana_HO: Keysym = Keysym(0x000004ce);
    /// kana_MA
    pub const KEY_kana_MA: Keysym = Keysym(0x000004cf);
    /// kana_MI
    pub const KEY_kana_MI: Keysym = Keysym(0x000004d0);
    /// kana_MU
    pub const KEY_kana_MU: Keysym = Keysym(0x000004d1);
    /// kana_ME
    pub const KEY_kana_ME: Keysym = Keysym(0x000004d2);
    /// kana_MO
    pub const KEY_kana_MO: Keysym = Keysym(0x000004d3);
    /// kana_YA
    pub const KEY_kana_YA: Keysym = Keysym(0x000004d4);
    /// kana_YU
    pub const KEY_kana_YU: Keysym = Keysym(0x000004d5);
    /// kana_YO
    pub const KEY_kana_YO: Keysym = Keysym(0x000004d6);
    /// kana_RA
    pub const KEY_kana_RA: Keysym = Keysym(0x000004d7);
    /// kana_RI
    pub const KEY_kana_RI: Keysym = Keysym(0x000004d8);
    /// kana_RU
    pub const KEY_kana_RU: Keysym = Keysym(0x000004d9);
    /// kana_RE
    pub const KEY_kana_RE: Keysym = Keysym(0x000004da);
    /// kana_RO
    pub const KEY_kana_RO: Keysym = Keysym(0x000004db);
    /// kana_WA
    pub const KEY_kana_WA: Keysym = Keysym(0x000004dc);
    /// kana_N
    pub const KEY_kana_N: Keysym = Keysym(0x000004dd);
    /// voicedsound
    pub const KEY_voicedsound: Keysym = Keysym(0x000004de);
    /// semivoicedsound
    pub const KEY_semivoicedsound: Keysym = Keysym(0x000004df);
    /// Farsi_0
    pub const KEY_Farsi_0: Keysym = Keysym(0x010006f0);
    /// Farsi_1
    pub const KEY_Farsi_1: Keysym = Keysym(0x010006f1);
    /// Farsi_2
    pub const KEY_Farsi_2: Keysym = Keysym(0x010006f2);
    /// Farsi_3
    pub const KEY_Farsi_3: Keysym = Keysym(0x010006f3);
    /// Farsi_4
    pub const KEY_Farsi_4: Keysym = Keysym(0x010006f4);
    /// Farsi_5
    pub const KEY_Farsi_5: Keysym = Keysym(0x010006f5);
    /// Farsi_6
    pub const KEY_Farsi_6: Keysym = Keysym(0x010006f6);
    /// Farsi_7
    pub const KEY_Farsi_7: Keysym = Keysym(0x010006f7);
    /// Farsi_8
    pub const KEY_Farsi_8: Keysym = Keysym(0x010006f8);
    /// Farsi_9
    pub const KEY_Farsi_9: Keysym = Keysym(0x010006f9);
    /// Arabic_percent
    pub const KEY_Arabic_percent: Keysym = Keysym(0x0100066a);
    /// Arabic_superscript_alef
    pub const KEY_Arabic_superscript_alef: Keysym = Keysym(0x01000670);
    /// Arabic_tteh
    pub const KEY_Arabic_tteh: Keysym = Keysym(0x01000679);
    /// Arabic_peh
    pub const KEY_Arabic_peh: Keysym = Keysym(0x0100067e);
    /// Arabic_tcheh
    pub const KEY_Arabic_tcheh: Keysym = Keysym(0x01000686);
    /// Arabic_ddal
    pub const KEY_Arabic_ddal: Keysym = Keysym(0x01000688);
    /// Arabic_rreh
    pub const KEY_Arabic_rreh: Keysym = Keysym(0x01000691);
    /// Arabic_comma
    pub const KEY_Arabic_comma: Keysym = Keysym(0x000005ac);
    /// Arabic_fullstop
    pub const KEY_Arabic_fullstop: Keysym = Keysym(0x010006d4);
    /// Arabic_0
    pub const KEY_Arabic_0: Keysym = Keysym(0x01000660);
    /// Arabic_1
    pub const KEY_Arabic_1: Keysym = Keysym(0x01000661);
    /// Arabic_2
    pub const KEY_Arabic_2: Keysym = Keysym(0x01000662);
    /// Arabic_3
    pub const KEY_Arabic_3: Keysym = Keysym(0x01000663);
    /// Arabic_4
    pub const KEY_Arabic_4: Keysym = Keysym(0x01000664);
    /// Arabic_5
    pub const KEY_Arabic_5: Keysym = Keysym(0x01000665);
    /// Arabic_6
    pub const KEY_Arabic_6: Keysym = Keysym(0x01000666);
    /// Arabic_7
    pub const KEY_Arabic_7: Keysym = Keysym(0x01000667);
    /// Arabic_8
    pub const KEY_Arabic_8: Keysym = Keysym(0x01000668);
    /// Arabic_9
    pub const KEY_Arabic_9: Keysym = Keysym(0x01000669);
    /// Arabic_semicolon
    pub const KEY_Arabic_semicolon: Keysym = Keysym(0x000005bb);
    /// Arabic_question_mark
    pub const KEY_Arabic_question_mark: Keysym = Keysym(0x000005bf);
    /// Arabic_hamza
    pub const KEY_Arabic_hamza: Keysym = Keysym(0x000005c1);
    /// Arabic_maddaonalef
    pub const KEY_Arabic_maddaonalef: Keysym = Keysym(0x000005c2);
    /// Arabic_hamzaonalef
    pub const KEY_Arabic_hamzaonalef: Keysym = Keysym(0x000005c3);
    /// Arabic_hamzaonwaw
    pub const KEY_Arabic_hamzaonwaw: Keysym = Keysym(0x000005c4);
    /// Arabic_hamzaunderalef
    pub const KEY_Arabic_hamzaunderalef: Keysym = Keysym(0x000005c5);
    /// Arabic_hamzaonyeh
    pub const KEY_Arabic_hamzaonyeh: Keysym = Keysym(0x000005c6);
    /// Arabic_alef
    pub const KEY_Arabic_alef: Keysym = Keysym(0x000005c7);
    /// Arabic_beh
    pub const KEY_Arabic_beh: Keysym = Keysym(0x000005c8);
    /// Arabic_tehmarbuta
    pub const KEY_Arabic_tehmarbuta: Keysym = Keysym(0x000005c9);
    /// Arabic_teh
    pub const KEY_Arabic_teh: Keysym = Keysym(0x000005ca);
    /// Arabic_theh
    pub const KEY_Arabic_theh: Keysym = Keysym(0x000005cb);
    /// Arabic_jeem
    pub const KEY_Arabic_jeem: Keysym = Keysym(0x000005cc);
    /// Arabic_hah
    pub const KEY_Arabic_hah: Keysym = Keysym(0x000005cd);
    /// Arabic_khah
    pub const KEY_Arabic_khah: Keysym = Keysym(0x000005ce);
    /// Arabic_dal
    pub const KEY_Arabic_dal: Keysym = Keysym(0x000005cf);
    /// Arabic_thal
    pub const KEY_Arabic_thal: Keysym = Keysym(0x000005d0);
    /// Arabic_ra
    pub const KEY_Arabic_ra: Keysym = Keysym(0x000005d1);
    /// Arabic_zain
    pub const KEY_Arabic_zain: Keysym = Keysym(0x000005d2);
    /// Arabic_seen
    pub const KEY_Arabic_seen: Keysym = Keysym(0x000005d3);
    /// Arabic_sheen
    pub const KEY_Arabic_sheen: Keysym = Keysym(0x000005d4);
    /// Arabic_sad
    pub const KEY_Arabic_sad: Keysym = Keysym(0x000005d5);
    /// Arabic_dad
    pub const KEY_Arabic_dad: Keysym = Keysym(0x000005d6);
    /// Arabic_tah
    pub const KEY_Arabic_tah: Keysym = Keysym(0x000005d7);
    /// Arabic_zah
    pub const KEY_Arabic_zah: Keysym = Keysym(0x000005d8);
    /// Arabic_ain
    pub const KEY_Arabic_ain: Keysym = Keysym(0x000005d9);
    /// Arabic_ghain
    pub const KEY_Arabic_ghain: Keysym = Keysym(0x000005da);
    /// Arabic_tatweel
    pub const KEY_Arabic_tatweel: Keysym = Keysym(0x000005e0);
    /// Arabic_feh
    pub const KEY_Arabic_feh: Keysym = Keysym(0x000005e1);
    /// Arabic_qaf
    pub const KEY_Arabic_qaf: Keysym = Keysym(0x000005e2);
    /// Arabic_kaf
    pub const KEY_Arabic_kaf: Keysym = Keysym(0x000005e3);
    /// Arabic_lam
    pub const KEY_Arabic_lam: Keysym = Keysym(0x000005e4);
    /// Arabic_meem
    pub const KEY_Arabic_meem: Keysym = Keysym(0x000005e5);
    /// Arabic_noon
    pub const KEY_Arabic_noon: Keysym = Keysym(0x000005e6);
    /// Arabic_ha
    pub const KEY_Arabic_ha: Keysym = Keysym(0x000005e7);
    /// Arabic_heh
    pub const KEY_Arabic_heh: Keysym = Keysym(0x000005e7);
    /// Arabic_waw
    pub const KEY_Arabic_waw: Keysym = Keysym(0x000005e8);
    /// Arabic_alefmaksura
    pub const KEY_Arabic_alefmaksura: Keysym = Keysym(0x000005e9);
    /// Arabic_yeh
    pub const KEY_Arabic_yeh: Keysym = Keysym(0x000005ea);
    /// Arabic_fathatan
    pub const KEY_Arabic_fathatan: Keysym = Keysym(0x000005eb);
    /// Arabic_dammatan
    pub const KEY_Arabic_dammatan: Keysym = Keysym(0x000005ec);
    /// Arabic_kasratan
    pub const KEY_Arabic_kasratan: Keysym = Keysym(0x000005ed);
    /// Arabic_fatha
    pub const KEY_Arabic_fatha: Keysym = Keysym(0x000005ee);
    /// Arabic_damma
    pub const KEY_Arabic_damma: Keysym = Keysym(0x000005ef);
    /// Arabic_kasra
    pub const KEY_Arabic_kasra: Keysym = Keysym(0x000005f0);
    /// Arabic_shadda
    pub const KEY_Arabic_shadda: Keysym = Keysym(0x000005f1);
    /// Arabic_sukun
    pub const KEY_Arabic_sukun: Keysym = Keysym(0x000005f2);
    /// Arabic_madda_above
    pub const KEY_Arabic_madda_above: Keysym = Keysym(0x01000653);
    /// Arabic_hamza_above
    pub const KEY_Arabic_hamza_above: Keysym = Keysym(0x01000654);
    /// Arabic_hamza_below
    pub const KEY_Arabic_hamza_below: Keysym = Keysym(0x01000655);
    /// Arabic_jeh
    pub const KEY_Arabic_jeh: Keysym = Keysym(0x01000698);
    /// Arabic_veh
    pub const KEY_Arabic_veh: Keysym = Keysym(0x010006a4);
    /// Arabic_keheh
    pub const KEY_Arabic_keheh: Keysym = Keysym(0x010006a9);
    /// Arabic_gaf
    pub const KEY_Arabic_gaf: Keysym = Keysym(0x010006af);
    /// Arabic_noon_ghunna
    pub const KEY_Arabic_noon_ghunna: Keysym = Keysym(0x010006ba);
    /// Arabic_heh_doachashmee
    pub const KEY_Arabic_heh_doachashmee: Keysym = Keysym(0x010006be);
    /// Farsi_yeh
    pub const KEY_Farsi_yeh: Keysym = Keysym(0x010006cc);
    /// Arabic_farsi_yeh
    pub const KEY_Arabic_farsi_yeh: Keysym = Keysym(0x010006cc);
    /// Arabic_yeh_baree
    pub const KEY_Arabic_yeh_baree: Keysym = Keysym(0x010006d2);
    /// Arabic_heh_goal
    pub const KEY_Arabic_heh_goal: Keysym = Keysym(0x010006c1);
    /// Cyrillic_GHE_bar
    pub const KEY_Cyrillic_GHE_bar: Keysym = Keysym(0x01000492);
    /// Cyrillic_ghe_bar
    pub const KEY_Cyrillic_ghe_bar: Keysym = Keysym(0x01000493);
    /// Cyrillic_ZHE_descender
    pub const KEY_Cyrillic_ZHE_descender: Keysym = Keysym(0x01000496);
    /// Cyrillic_zhe_descender
    pub const KEY_Cyrillic_zhe_descender: Keysym = Keysym(0x01000497);
    /// Cyrillic_KA_descender
    pub const KEY_Cyrillic_KA_descender: Keysym = Keysym(0x0100049a);
    /// Cyrillic_ka_descender
    pub const KEY_Cyrillic_ka_descender: Keysym = Keysym(0x0100049b);
    /// Cyrillic_KA_vertstroke
    pub const KEY_Cyrillic_KA_vertstroke: Keysym = Keysym(0x0100049c);
    /// Cyrillic_ka_vertstroke
    pub const KEY_Cyrillic_ka_vertstroke: Keysym = Keysym(0x0100049d);
    /// Cyrillic_EN_descender
    pub const KEY_Cyrillic_EN_descender: Keysym = Keysym(0x010004a2);
    /// Cyrillic_en_descender
    pub const KEY_Cyrillic_en_descender: Keysym = Keysym(0x010004a3);
    /// Cyrillic_U_straight
    pub const KEY_Cyrillic_U_straight: Keysym = Keysym(0x010004ae);
    /// Cyrillic_u_straight
    pub const KEY_Cyrillic_u_straight: Keysym = Keysym(0x010004af);
    /// Cyrillic_U_straight_bar
    pub const KEY_Cyrillic_U_straight_bar: Keysym = Keysym(0x010004b0);
    /// Cyrillic_u_straight_bar
    pub const KEY_Cyrillic_u_straight_bar: Keysym = Keysym(0x010004b1);
    /// Cyrillic_HA_descender
    pub const KEY_Cyrillic_HA_descender: Keysym = Keysym(0x010004b2);
    /// Cyrillic_ha_descender
    pub const KEY_Cyrillic_ha_descender: Keysym = Keysym(0x010004b3);
    /// Cyrillic_CHE_descender
    pub const KEY_Cyrillic_CHE_descender: Keysym = Keysym(0x010004b6);
    /// Cyrillic_che_descender
    pub const KEY_Cyrillic_che_descender: Keysym = Keysym(0x010004b7);
    /// Cyrillic_CHE_vertstroke
    pub const KEY_Cyrillic_CHE_vertstroke: Keysym = Keysym(0x010004b8);
    /// Cyrillic_che_vertstroke
    pub const KEY_Cyrillic_che_vertstroke: Keysym = Keysym(0x010004b9);
    /// Cyrillic_SHHA
    pub const KEY_Cyrillic_SHHA: Keysym = Keysym(0x010004ba);
    /// Cyrillic_shha
    pub const KEY_Cyrillic_shha: Keysym = Keysym(0x010004bb);
    /// Cyrillic_SCHWA
    pub const KEY_Cyrillic_SCHWA: Keysym = Keysym(0x010004d8);
    /// Cyrillic_schwa
    pub const KEY_Cyrillic_schwa: Keysym = Keysym(0x010004d9);
    /// Cyrillic_I_macron
    pub const KEY_Cyrillic_I_macron: Keysym = Keysym(0x010004e2);
    /// Cyrillic_i_macron
    pub const KEY_Cyrillic_i_macron: Keysym = Keysym(0x010004e3);
    /// Cyrillic_O_bar
    pub const KEY_Cyrillic_O_bar: Keysym = Keysym(0x010004e8);
    /// Cyrillic_o_bar
    pub const KEY_Cyrillic_o_bar: Keysym = Keysym(0x010004e9);
    /// Cyrillic_U_macron
    pub const KEY_Cyrillic_U_macron: Keysym = Keysym(0x010004ee);
    /// Cyrillic_u_macron
    pub const KEY_Cyrillic_u_macron: Keysym = Keysym(0x010004ef);
    /// Serbian_dje
    pub const KEY_Serbian_dje: Keysym = Keysym(0x000006a1);
    /// Macedonia_gje
    pub const KEY_Macedonia_gje: Keysym = Keysym(0x000006a2);
    /// Cyrillic_io
    pub const KEY_Cyrillic_io: Keysym = Keysym(0x000006a3);
    /// Ukrainian_ie
    pub const KEY_Ukrainian_ie: Keysym = Keysym(0x000006a4);
    /// Ukranian_je
    pub const KEY_Ukranian_je: Keysym = Keysym(0x000006a4);
    /// Macedonia_dse
    pub const KEY_Macedonia_dse: Keysym = Keysym(0x000006a5);
    /// Ukrainian_i
    pub const KEY_Ukrainian_i: Keysym = Keysym(0x000006a6);
    /// Ukranian_i
    pub const KEY_Ukranian_i: Keysym = Keysym(0x000006a6);
    /// Ukrainian_yi
    pub const KEY_Ukrainian_yi: Keysym = Keysym(0x000006a7);
    /// Ukranian_yi
    pub const KEY_Ukranian_yi: Keysym = Keysym(0x000006a7);
    /// Cyrillic_je
    pub const KEY_Cyrillic_je: Keysym = Keysym(0x000006a8);
    /// Serbian_je
    pub const KEY_Serbian_je: Keysym = Keysym(0x000006a8);
    /// Cyrillic_lje
    pub const KEY_Cyrillic_lje: Keysym = Keysym(0x000006a9);
    /// Serbian_lje
    pub const KEY_Serbian_lje: Keysym = Keysym(0x000006a9);
    /// Cyrillic_nje
    pub const KEY_Cyrillic_nje: Keysym = Keysym(0x000006aa);
    /// Serbian_nje
    pub const KEY_Serbian_nje: Keysym = Keysym(0x000006aa);
    /// Serbian_tshe
    pub const KEY_Serbian_tshe: Keysym = Keysym(0x000006ab);
    /// Macedonia_kje
    pub const KEY_Macedonia_kje: Keysym = Keysym(0x000006ac);
    /// Ukrainian_ghe_with_upturn
    pub const KEY_Ukrainian_ghe_with_upturn: Keysym = Keysym(0x000006ad);
    /// Byelorussian_shortu
    pub const KEY_Byelorussian_shortu: Keysym = Keysym(0x000006ae);
    /// Cyrillic_dzhe
    pub const KEY_Cyrillic_dzhe: Keysym = Keysym(0x000006af);
    /// Serbian_dze
    pub const KEY_Serbian_dze: Keysym = Keysym(0x000006af);
    /// numerosign
    pub const KEY_numerosign: Keysym = Keysym(0x000006b0);
    /// Serbian_DJE
    pub const KEY_Serbian_DJE: Keysym = Keysym(0x000006b1);
    /// Macedonia_GJE
    pub const KEY_Macedonia_GJE: Keysym = Keysym(0x000006b2);
    /// Cyrillic_IO
    pub const KEY_Cyrillic_IO: Keysym = Keysym(0x000006b3);
    /// Ukrainian_IE
    pub const KEY_Ukrainian_IE: Keysym = Keysym(0x000006b4);
    /// Ukranian_JE
    pub const KEY_Ukranian_JE: Keysym = Keysym(0x000006b4);
    /// Macedonia_DSE
    pub const KEY_Macedonia_DSE: Keysym = Keysym(0x000006b5);
    /// Ukrainian_I
    pub const KEY_Ukrainian_I: Keysym = Keysym(0x000006b6);
    /// Ukranian_I
    pub const KEY_Ukranian_I: Keysym = Keysym(0x000006b6);
    /// Ukrainian_YI
    pub const KEY_Ukrainian_YI: Keysym = Keysym(0x000006b7);
    /// Ukranian_YI
    pub const KEY_Ukranian_YI: Keysym = Keysym(0x000006b7);
    /// Cyrillic_JE
    pub const KEY_Cyrillic_JE: Keysym = Keysym(0x000006b8);
    /// Serbian_JE
    pub const KEY_Serbian_JE: Keysym = Keysym(0x000006b8);
    /// Cyrillic_LJE
    pub const KEY_Cyrillic_LJE: Keysym = Keysym(0x000006b9);
    /// Serbian_LJE
    pub const KEY_Serbian_LJE: Keysym = Keysym(0x000006b9);
    /// Cyrillic_NJE
    pub const KEY_Cyrillic_NJE: Keysym = Keysym(0x000006ba);
    /// Serbian_NJE
    pub const KEY_Serbian_NJE: Keysym = Keysym(0x000006ba);
    /// Serbian_TSHE
    pub const KEY_Serbian_TSHE: Keysym = Keysym(0x000006bb);
    /// Macedonia_KJE
    pub const KEY_Macedonia_KJE: Keysym = Keysym(0x000006bc);
    /// Ukrainian_GHE_WITH_UPTURN
    pub const KEY_Ukrainian_GHE_WITH_UPTURN: Keysym = Keysym(0x000006bd);
    /// Byelorussian_SHORTU
    pub const KEY_Byelorussian_SHORTU: Keysym = Keysym(0x000006be);
    /// Cyrillic_DZHE
    pub const KEY_Cyrillic_DZHE: Keysym = Keysym(0x000006bf);
    /// Serbian_DZE
    pub const KEY_Serbian_DZE: Keysym = Keysym(0x000006bf);
    /// Cyrillic_yu
    pub const KEY_Cyrillic_yu: Keysym = Keysym(0x000006c0);
    /// Cyrillic_a
    pub const KEY_Cyrillic_a: Keysym = Keysym(0x000006c1);
    /// Cyrillic_be
    pub const KEY_Cyrillic_be: Keysym = Keysym(0x000006c2);
    /// Cyrillic_tse
    pub const KEY_Cyrillic_tse: Keysym = Keysym(0x000006c3);
    /// Cyrillic_de
    pub const KEY_Cyrillic_de: Keysym = Keysym(0x000006c4);
    /// Cyrillic_ie
    pub const KEY_Cyrillic_ie: Keysym = Keysym(0x000006c5);
    /// Cyrillic_ef
    pub const KEY_Cyrillic_ef: Keysym = Keysym(0x000006c6);
    /// Cyrillic_ghe
    pub const KEY_Cyrillic_ghe: Keysym = Keysym(0x000006c7);
    /// Cyrillic_ha
    pub const KEY_Cyrillic_ha: Keysym = Keysym(0x000006c8);
    /// Cyrillic_i
    pub const KEY_Cyrillic_i: Keysym = Keysym(0x000006c9);
    /// Cyrillic_shorti
    pub const KEY_Cyrillic_shorti: Keysym = Keysym(0x000006ca);
    /// Cyrillic_ka
    pub const KEY_Cyrillic_ka: Keysym = Keysym(0x000006cb);
    /// Cyrillic_el
    pub const KEY_Cyrillic_el: Keysym = Keysym(0x000006cc);
    /// Cyrillic_em
    pub const KEY_Cyrillic_em: Keysym = Keysym(0x000006cd);
    /// Cyrillic_en
    pub const KEY_Cyrillic_en: Keysym = Keysym(0x000006ce);
    /// Cyrillic_o
    pub const KEY_Cyrillic_o: Keysym = Keysym(0x000006cf);
    /// Cyrillic_pe
    pub const KEY_Cyrillic_pe: Keysym = Keysym(0x000006d0);
    /// Cyrillic_ya
    pub const KEY_Cyrillic_ya: Keysym = Keysym(0x000006d1);
    /// Cyrillic_er
    pub const KEY_Cyrillic_er: Keysym = Keysym(0x000006d2);
    /// Cyrillic_es
    pub const KEY_Cyrillic_es: Keysym = Keysym(0x000006d3);
    /// Cyrillic_te
    pub const KEY_Cyrillic_te: Keysym = Keysym(0x000006d4);
    /// Cyrillic_u
    pub const KEY_Cyrillic_u: Keysym = Keysym(0x000006d5);
    /// Cyrillic_zhe
    pub const KEY_Cyrillic_zhe: Keysym = Keysym(0x000006d6);
    /// Cyrillic_ve
    pub const KEY_Cyrillic_ve: Keysym = Keysym(0x000006d7);
    /// Cyrillic_softsign
    pub const KEY_Cyrillic_softsign: Keysym = Keysym(0x000006d8);
    /// Cyrillic_yeru
    pub const KEY_Cyrillic_yeru: Keysym = Keysym(0x000006d9);
    /// Cyrillic_ze
    pub const KEY_Cyrillic_ze: Keysym = Keysym(0x000006da);
    /// Cyrillic_sha
    pub const KEY_Cyrillic_sha: Keysym = Keysym(0x000006db);
    /// Cyrillic_e
    pub const KEY_Cyrillic_e: Keysym = Keysym(0x000006dc);
    /// Cyrillic_shcha
    pub const KEY_Cyrillic_shcha: Keysym = Keysym(0x000006dd);
    /// Cyrillic_che
    pub const KEY_Cyrillic_che: Keysym = Keysym(0x000006de);
    /// Cyrillic_hardsign
    pub const KEY_Cyrillic_hardsign: Keysym = Keysym(0x000006df);
    /// Cyrillic_YU
    pub const KEY_Cyrillic_YU: Keysym = Keysym(0x000006e0);
    /// Cyrillic_A
    pub const KEY_Cyrillic_A: Keysym = Keysym(0x000006e1);
    /// Cyrillic_BE
    pub const KEY_Cyrillic_BE: Keysym = Keysym(0x000006e2);
    /// Cyrillic_TSE
    pub const KEY_Cyrillic_TSE: Keysym = Keysym(0x000006e3);
    /// Cyrillic_DE
    pub const KEY_Cyrillic_DE: Keysym = Keysym(0x000006e4);
    /// Cyrillic_IE
    pub const KEY_Cyrillic_IE: Keysym = Keysym(0x000006e5);
    /// Cyrillic_EF
    pub const KEY_Cyrillic_EF: Keysym = Keysym(0x000006e6);
    /// Cyrillic_GHE
    pub const KEY_Cyrillic_GHE: Keysym = Keysym(0x000006e7);
    /// Cyrillic_HA
    pub const KEY_Cyrillic_HA: Keysym = Keysym(0x000006e8);
    /// Cyrillic_I
    pub const KEY_Cyrillic_I: Keysym = Keysym(0x000006e9);
    /// Cyrillic_SHORTI
    pub const KEY_Cyrillic_SHORTI: Keysym = Keysym(0x000006ea);
    /// Cyrillic_KA
    pub const KEY_Cyrillic_KA: Keysym = Keysym(0x000006eb);
    /// Cyrillic_EL
    pub const KEY_Cyrillic_EL: Keysym = Keysym(0x000006ec);
    /// Cyrillic_EM
    pub const KEY_Cyrillic_EM: Keysym = Keysym(0x000006ed);
    /// Cyrillic_EN
    pub const KEY_Cyrillic_EN: Keysym = Keysym(0x000006ee);
    /// Cyrillic_O
    pub const KEY_Cyrillic_O: Keysym = Keysym(0x000006ef);
    /// Cyrillic_PE
    pub const KEY_Cyrillic_PE: Keysym = Keysym(0x000006f0);
    /// Cyrillic_YA
    pub const KEY_Cyrillic_YA: Keysym = Keysym(0x000006f1);
    /// Cyrillic_ER
    pub const KEY_Cyrillic_ER: Keysym = Keysym(0x000006f2);
    /// Cyrillic_ES
    pub const KEY_Cyrillic_ES: Keysym = Keysym(0x000006f3);
    /// Cyrillic_TE
    pub const KEY_Cyrillic_TE: Keysym = Keysym(0x000006f4);
    /// Cyrillic_U
    pub const KEY_Cyrillic_U: Keysym = Keysym(0x000006f5);
    /// Cyrillic_ZHE
    pub const KEY_Cyrillic_ZHE: Keysym = Keysym(0x000006f6);
    /// Cyrillic_VE
    pub const KEY_Cyrillic_VE: Keysym = Keysym(0x000006f7);
    /// Cyrillic_SOFTSIGN
    pub const KEY_Cyrillic_SOFTSIGN: Keysym = Keysym(0x000006f8);
    /// Cyrillic_YERU
    pub const KEY_Cyrillic_YERU: Keysym = Keysym(0x000006f9);
    /// Cyrillic_ZE
    pub const KEY_Cyrillic_ZE: Keysym = Keysym(0x000006fa);
    /// Cyrillic_SHA
    pub const KEY_Cyrillic_SHA: Keysym = Keysym(0x000006fb);
    /// Cyrillic_E
    pub const KEY_Cyrillic_E: Keysym = Keysym(0x000006fc);
    /// Cyrillic_SHCHA
    pub const KEY_Cyrillic_SHCHA: Keysym = Keysym(0x000006fd);
    /// Cyrillic_CHE
    pub const KEY_Cyrillic_CHE: Keysym = Keysym(0x000006fe);
    /// Cyrillic_HARDSIGN
    pub const KEY_Cyrillic_HARDSIGN: Keysym = Keysym(0x000006ff);
    /// Greek_ALPHAaccent
    pub const KEY_Greek_ALPHAaccent: Keysym = Keysym(0x000007a1);
    /// Greek_EPSILONaccent
    pub const KEY_Greek_EPSILONaccent: Keysym = Keysym(0x000007a2);
    /// Greek_ETAaccent
    pub const KEY_Greek_ETAaccent: Keysym = Keysym(0x000007a3);
    /// Greek_IOTAaccent
    pub const KEY_Greek_IOTAaccent: Keysym = Keysym(0x000007a4);
    /// Greek_IOTAdieresis
    pub const KEY_Greek_IOTAdieresis: Keysym = Keysym(0x000007a5);
    /// Greek_IOTAdiaeresis
    pub const KEY_Greek_IOTAdiaeresis: Keysym = Keysym(0x000007a5);
    /// Greek_OMICRONaccent
    pub const KEY_Greek_OMICRONaccent: Keysym = Keysym(0x000007a7);
    /// Greek_UPSILONaccent
    pub const KEY_Greek_UPSILONaccent: Keysym = Keysym(0x000007a8);
    /// Greek_UPSILONdieresis
    pub const KEY_Greek_UPSILONdieresis: Keysym = Keysym(0x000007a9);
    /// Greek_OMEGAaccent
    pub const KEY_Greek_OMEGAaccent: Keysym = Keysym(0x000007ab);
    /// Greek_accentdieresis
    pub const KEY_Greek_accentdieresis: Keysym = Keysym(0x000007ae);
    /// Greek_horizbar
    pub const KEY_Greek_horizbar: Keysym = Keysym(0x000007af);
    /// Greek_alphaaccent
    pub const KEY_Greek_alphaaccent: Keysym = Keysym(0x000007b1);
    /// Greek_epsilonaccent
    pub const KEY_Greek_epsilonaccent: Keysym = Keysym(0x000007b2);
    /// Greek_etaaccent
    pub const KEY_Greek_etaaccent: Keysym = Keysym(0x000007b3);
    /// Greek_iotaaccent
    pub const KEY_Greek_iotaaccent: Keysym = Keysym(0x000007b4);
    /// Greek_iotadieresis
    pub const KEY_Greek_iotadieresis: Keysym = Keysym(0x000007b5);
    /// Greek_iotaaccentdieresis
    pub const KEY_Greek_iotaaccentdieresis: Keysym = Keysym(0x000007b6);
    /// Greek_omicronaccent
    pub const KEY_Greek_omicronaccent: Keysym = Keysym(0x000007b7);
    /// Greek_upsilonaccent
    pub const KEY_Greek_upsilonaccent: Keysym = Keysym(0x000007b8);
    /// Greek_upsilondieresis
    pub const KEY_Greek_upsilondieresis: Keysym = Keysym(0x000007b9);
    /// Greek_upsilonaccentdieresis
    pub const KEY_Greek_upsilonaccentdieresis: Keysym = Keysym(0x000007ba);
    /// Greek_omegaaccent
    pub const KEY_Greek_omegaaccent: Keysym = Keysym(0x000007bb);
    /// Greek_ALPHA
    pub const KEY_Greek_ALPHA: Keysym = Keysym(0x000007c1);
    /// Greek_BETA
    pub const KEY_Greek_BETA: Keysym = Keysym(0x000007c2);
    /// Greek_GAMMA
    pub const KEY_Greek_GAMMA: Keysym = Keysym(0x000007c3);
    /// Greek_DELTA
    pub const KEY_Greek_DELTA: Keysym = Keysym(0x000007c4);
    /// Greek_EPSILON
    pub const KEY_Greek_EPSILON: Keysym = Keysym(0x000007c5);
    /// Greek_ZETA
    pub const KEY_Greek_ZETA: Keysym = Keysym(0x000007c6);
    /// Greek_ETA
    pub const KEY_Greek_ETA: Keysym = Keysym(0x000007c7);
    /// Greek_THETA
    pub const KEY_Greek_THETA: Keysym = Keysym(0x000007c8);
    /// Greek_IOTA
    pub const KEY_Greek_IOTA: Keysym = Keysym(0x000007c9);
    /// Greek_KAPPA
    pub const KEY_Greek_KAPPA: Keysym = Keysym(0x000007ca);
    /// Greek_LAMDA
    pub const KEY_Greek_LAMDA: Keysym = Keysym(0x000007cb);
    /// Greek_LAMBDA
    pub const KEY_Greek_LAMBDA: Keysym = Keysym(0x000007cb);
    /// Greek_MU
    pub const KEY_Greek_MU: Keysym = Keysym(0x000007cc);
    /// Greek_NU
    pub const KEY_Greek_NU: Keysym = Keysym(0x000007cd);
    /// Greek_XI
    pub const KEY_Greek_XI: Keysym = Keysym(0x000007ce);
    /// Greek_OMICRON
    pub const KEY_Greek_OMICRON: Keysym = Keysym(0x000007cf);
    /// Greek_PI
    pub const KEY_Greek_PI: Keysym = Keysym(0x000007d0);
    /// Greek_RHO
    pub const KEY_Greek_RHO: Keysym = Keysym(0x000007d1);
    /// Greek_SIGMA
    pub const KEY_Greek_SIGMA: Keysym = Keysym(0x000007d2);
    /// Greek_TAU
    pub const KEY_Greek_TAU: Keysym = Keysym(0x000007d4);
    /// Greek_UPSILON
    pub const KEY_Greek_UPSILON: Keysym = Keysym(0x000007d5);
    /// Greek_PHI
    pub const KEY_Greek_PHI: Keysym = Keysym(0x000007d6);
    /// Greek_CHI
    pub const KEY_Greek_CHI: Keysym = Keysym(0x000007d7);
    /// Greek_PSI
    pub const KEY_Greek_PSI: Keysym = Keysym(0x000007d8);
    /// Greek_OMEGA
    pub const KEY_Greek_OMEGA: Keysym = Keysym(0x000007d9);
    /// Greek_alpha
    pub const KEY_Greek_alpha: Keysym = Keysym(0x000007e1);
    /// Greek_beta
    pub const KEY_Greek_beta: Keysym = Keysym(0x000007e2);
    /// Greek_gamma
    pub const KEY_Greek_gamma: Keysym = Keysym(0x000007e3);
    /// Greek_delta
    pub const KEY_Greek_delta: Keysym = Keysym(0x000007e4);
    /// Greek_epsilon
    pub const KEY_Greek_epsilon: Keysym = Keysym(0x000007e5);
    /// Greek_zeta
    pub const KEY_Greek_zeta: Keysym = Keysym(0x000007e6);
    /// Greek_eta
    pub const KEY_Greek_eta: Keysym = Keysym(0x000007e7);
    /// Greek_theta
    pub const KEY_Greek_theta: Keysym = Keysym(0x000007e8);
    /// Greek_iota
    pub const KEY_Greek_iota: Keysym = Keysym(0x000007e9);
    /// Greek_kappa
    pub const KEY_Greek_kappa: Keysym = Keysym(0x000007ea);
    /// Greek_lamda
    pub const KEY_Greek_lamda: Keysym = Keysym(0x000007eb);
    /// Greek_lambda
    pub const KEY_Greek_lambda: Keysym = Keysym(0x000007eb);
    /// Greek_mu
    pub const KEY_Greek_mu: Keysym = Keysym(0x000007ec);
    /// Greek_nu
    pub const KEY_Greek_nu: Keysym = Keysym(0x000007ed);
    /// Greek_xi
    pub const KEY_Greek_xi: Keysym = Keysym(0x000007ee);
    /// Greek_omicron
    pub const KEY_Greek_omicron: Keysym = Keysym(0x000007ef);
    /// Greek_pi
    pub const KEY_Greek_pi: Keysym = Keysym(0x000007f0);
    /// Greek_rho
    pub const KEY_Greek_rho: Keysym = Keysym(0x000007f1);
    /// Greek_sigma
    pub const KEY_Greek_sigma: Keysym = Keysym(0x000007f2);
    /// Greek_finalsmallsigma
    pub const KEY_Greek_finalsmallsigma: Keysym = Keysym(0x000007f3);
    /// Greek_tau
    pub const KEY_Greek_tau: Keysym = Keysym(0x000007f4);
    /// Greek_upsilon
    pub const KEY_Greek_upsilon: Keysym = Keysym(0x000007f5);
    /// Greek_phi
    pub const KEY_Greek_phi: Keysym = Keysym(0x000007f6);
    /// Greek_chi
    pub const KEY_Greek_chi: Keysym = Keysym(0x000007f7);
    /// Greek_psi
    pub const KEY_Greek_psi: Keysym = Keysym(0x000007f8);
    /// Greek_omega
    pub const KEY_Greek_omega: Keysym = Keysym(0x000007f9);
    /// leftradical
    pub const KEY_leftradical: Keysym = Keysym(0x000008a1);
    /// topleftradical
    pub const KEY_topleftradical: Keysym = Keysym(0x000008a2);
    /// horizconnector
    pub const KEY_horizconnector: Keysym = Keysym(0x000008a3);
    /// topintegral
    pub const KEY_topintegral: Keysym = Keysym(0x000008a4);
    /// botintegral
    pub const KEY_botintegral: Keysym = Keysym(0x000008a5);
    /// vertconnector
    pub const KEY_vertconnector: Keysym = Keysym(0x000008a6);
    /// topleftsqbracket
    pub const KEY_topleftsqbracket: Keysym = Keysym(0x000008a7);
    /// botleftsqbracket
    pub const KEY_botleftsqbracket: Keysym = Keysym(0x000008a8);
    /// toprightsqbracket
    pub const KEY_toprightsqbracket: Keysym = Keysym(0x000008a9);
    /// botrightsqbracket
    pub const KEY_botrightsqbracket: Keysym = Keysym(0x000008aa);
    /// topleftparens
    pub const KEY_topleftparens: Keysym = Keysym(0x000008ab);
    /// botleftparens
    pub const KEY_botleftparens: Keysym = Keysym(0x000008ac);
    /// toprightparens
    pub const KEY_toprightparens: Keysym = Keysym(0x000008ad);
    /// botrightparens
    pub const KEY_botrightparens: Keysym = Keysym(0x000008ae);
    /// leftmiddlecurlybrace
    pub const KEY_leftmiddlecurlybrace: Keysym = Keysym(0x000008af);
    /// rightmiddlecurlybrace
    pub const KEY_rightmiddlecurlybrace: Keysym = Keysym(0x000008b0);
    /// topleftsummation
    pub const KEY_topleftsummation: Keysym = Keysym(0x000008b1);
    /// botleftsummation
    pub const KEY_botleftsummation: Keysym = Keysym(0x000008b2);
    /// topvertsummationconnector
    pub const KEY_topvertsummationconnector: Keysym = Keysym(0x000008b3);
    /// botvertsummationconnector
    pub const KEY_botvertsummationconnector: Keysym = Keysym(0x000008b4);
    /// toprightsummation
    pub const KEY_toprightsummation: Keysym = Keysym(0x000008b5);
    /// botrightsummation
    pub const KEY_botrightsummation: Keysym = Keysym(0x000008b6);
    /// rightmiddlesummation
    pub const KEY_rightmiddlesummation: Keysym = Keysym(0x000008b7);
    /// lessthanequal
    pub const KEY_lessthanequal: Keysym = Keysym(0x000008bc);
    /// notequal
    pub const KEY_notequal: Keysym = Keysym(0x000008bd);
    /// greaterthanequal
    pub const KEY_greaterthanequal: Keysym = Keysym(0x000008be);
    /// integral
    pub const KEY_integral: Keysym = Keysym(0x000008bf);
    /// therefore
    pub const KEY_therefore: Keysym = Keysym(0x000008c0);
    /// variation
    pub const KEY_variation: Keysym = Keysym(0x000008c1);
    /// infinity
    pub const KEY_infinity: Keysym = Keysym(0x000008c2);
    /// nabla
    pub const KEY_nabla: Keysym = Keysym(0x000008c5);
    /// approximate
    pub const KEY_approximate: Keysym = Keysym(0x000008c8);
    /// similarequal
    pub const KEY_similarequal: Keysym = Keysym(0x000008c9);
    /// ifonlyif
    pub const KEY_ifonlyif: Keysym = Keysym(0x000008cd);
    /// implies
    pub const KEY_implies: Keysym = Keysym(0x000008ce);
    /// identical
    pub const KEY_identical: Keysym = Keysym(0x000008cf);
    /// radical
    pub const KEY_radical: Keysym = Keysym(0x000008d6);
    /// includedin
    pub const KEY_includedin: Keysym = Keysym(0x000008da);
    /// includes
    pub const KEY_includes: Keysym = Keysym(0x000008db);
    /// intersection
    pub const KEY_intersection: Keysym = Keysym(0x000008dc);
    /// union
    pub const KEY_union: Keysym = Keysym(0x000008dd);
    /// logicaland
    pub const KEY_logicaland: Keysym = Keysym(0x000008de);
    /// logicalor
    pub const KEY_logicalor: Keysym = Keysym(0x000008df);
    /// partialderivative
    pub const KEY_partialderivative: Keysym = Keysym(0x000008ef);
    /// function
    pub const KEY_function: Keysym = Keysym(0x000008f6);
    /// leftarrow
    pub const KEY_leftarrow: Keysym = Keysym(0x000008fb);
    /// uparrow
    pub const KEY_uparrow: Keysym = Keysym(0x000008fc);
    /// rightarrow
    pub const KEY_rightarrow: Keysym = Keysym(0x000008fd);
    /// downarrow
    pub const KEY_downarrow: Keysym = Keysym(0x000008fe);
    /// blank
    pub const KEY_blank: Keysym = Keysym(0x000009df);
    /// soliddiamond
    pub const KEY_soliddiamond: Keysym = Keysym(0x000009e0);
    /// checkerboard
    pub const KEY_checkerboard: Keysym = Keysym(0x000009e1);
    /// ht
    pub const KEY_ht: Keysym = Keysym(0x000009e2);
    /// ff
    pub const KEY_ff: Keysym = Keysym(0x000009e3);
    /// cr
    pub const KEY_cr: Keysym = Keysym(0x000009e4);
    /// lf
    pub const KEY_lf: Keysym = Keysym(0x000009e5);
    /// nl
    pub const KEY_nl: Keysym = Keysym(0x000009e8);
    /// vt
    pub const KEY_vt: Keysym = Keysym(0x000009e9);
    /// lowrightcorner
    pub const KEY_lowrightcorner: Keysym = Keysym(0x000009ea);
    /// uprightcorner
    pub const KEY_uprightcorner: Keysym = Keysym(0x000009eb);
    /// upleftcorner
    pub const KEY_upleftcorner: Keysym = Keysym(0x000009ec);
    /// lowleftcorner
    pub const KEY_lowleftcorner: Keysym = Keysym(0x000009ed);
    /// crossinglines
    pub const KEY_crossinglines: Keysym = Keysym(0x000009ee);
    /// horizlinescan1
    pub const KEY_horizlinescan1: Keysym = Keysym(0x000009ef);
    /// horizlinescan3
    pub const KEY_horizlinescan3: Keysym = Keysym(0x000009f0);
    /// horizlinescan5
    pub const KEY_horizlinescan5: Keysym = Keysym(0x000009f1);
    /// horizlinescan7
    pub const KEY_horizlinescan7: Keysym = Keysym(0x000009f2);
    /// horizlinescan9
    pub const KEY_horizlinescan9: Keysym = Keysym(0x000009f3);
    /// leftt
    pub const KEY_leftt: Keysym = Keysym(0x000009f4);
    /// rightt
    pub const KEY_rightt: Keysym = Keysym(0x000009f5);
    /// bott
    pub const KEY_bott: Keysym = Keysym(0x000009f6);
    /// topt
    pub const KEY_topt: Keysym = Keysym(0x000009f7);
    /// vertbar
    pub const KEY_vertbar: Keysym = Keysym(0x000009f8);
    /// emspace
    pub const KEY_emspace: Keysym = Keysym(0x00000aa1);
    /// enspace
    pub const KEY_enspace: Keysym = Keysym(0x00000aa2);
    /// em3space
    pub const KEY_em3space: Keysym = Keysym(0x00000aa3);
    /// em4space
    pub const KEY_em4space: Keysym = Keysym(0x00000aa4);
    /// digitspace
    pub const KEY_digitspace: Keysym = Keysym(0x00000aa5);
    /// punctspace
    pub const KEY_punctspace: Keysym = Keysym(0x00000aa6);
    /// thinspace
    pub const KEY_thinspace: Keysym = Keysym(0x00000aa7);
    /// hairspace
    pub const KEY_hairspace: Keysym = Keysym(0x00000aa8);
    /// emdash
    pub const KEY_emdash: Keysym = Keysym(0x00000aa9);
    /// endash
    pub const KEY_endash: Keysym = Keysym(0x00000aaa);
    /// signifblank
    pub const KEY_signifblank: Keysym = Keysym(0x00000aac);
    /// ellipsis
    pub const KEY_ellipsis: Keysym = Keysym(0x00000aae);
    /// doubbaselinedot
    pub const KEY_doubbaselinedot: Keysym = Keysym(0x00000aaf);
    /// onethird
    pub const KEY_onethird: Keysym = Keysym(0x00000ab0);
    /// twothirds
    pub const KEY_twothirds: Keysym = Keysym(0x00000ab1);
    /// onefifth
    pub const KEY_onefifth: Keysym = Keysym(0x00000ab2);
    /// twofifths
    pub const KEY_twofifths: Keysym = Keysym(0x00000ab3);
    /// threefifths
    pub const KEY_threefifths: Keysym = Keysym(0x00000ab4);
    /// fourfifths
    pub const KEY_fourfifths: Keysym = Keysym(0x00000ab5);
    /// onesixth
    pub const KEY_onesixth: Keysym = Keysym(0x00000ab6);
    /// fivesixths
    pub const KEY_fivesixths: Keysym = Keysym(0x00000ab7);
    /// careof
    pub const KEY_careof: Keysym = Keysym(0x00000ab8);
    /// figdash
    pub const KEY_figdash: Keysym = Keysym(0x00000abb);
    /// leftanglebracket
    pub const KEY_leftanglebracket: Keysym = Keysym(0x00000abc);
    /// decimalpoint
    pub const KEY_decimalpoint: Keysym = Keysym(0x00000abd);
    /// rightanglebracket
    pub const KEY_rightanglebracket: Keysym = Keysym(0x00000abe);
    /// marker
    pub const KEY_marker: Keysym = Keysym(0x00000abf);
    /// oneeighth
    pub const KEY_oneeighth: Keysym = Keysym(0x00000ac3);
    /// threeeighths
    pub const KEY_threeeighths: Keysym = Keysym(0x00000ac4);
    /// fiveeighths
    pub const KEY_fiveeighths: Keysym = Keysym(0x00000ac5);
    /// seveneighths
    pub const KEY_seveneighths: Keysym = Keysym(0x00000ac6);
    /// trademark
    pub const KEY_trademark: Keysym = Keysym(0x00000ac9);
    /// signaturemark
    pub const KEY_signaturemark: Keysym = Keysym(0x00000aca);
    /// trademarkincircle
    pub const KEY_trademarkincircle: Keysym = Keysym(0x00000acb);
    /// leftopentriangle
    pub const KEY_leftopentriangle: Keysym = Keysym(0x00000acc);
    /// rightopentriangle
    pub const KEY_rightopentriangle: Keysym = Keysym(0x00000acd);
    /// emopencircle
    pub const KEY_emopencircle: Keysym = Keysym(0x00000ace);
    /// emopenrectangle
    pub const KEY_emopenrectangle: Keysym = Keysym(0x00000acf);
    /// leftsinglequotemark
    pub const KEY_leftsinglequotemark: Keysym = Keysym(0x00000ad0);
    /// rightsinglequotemark
    pub const KEY_rightsinglequotemark: Keysym = Keysym(0x00000ad1);
    /// leftdoublequotemark
    pub const KEY_leftdoublequotemark: Keysym = Keysym(0x00000ad2);
    /// rightdoublequotemark
    pub const KEY_rightdoublequotemark: Keysym = Keysym(0x00000ad3);
    /// prescription
    pub const KEY_prescription: Keysym = Keysym(0x00000ad4);
    /// permille
    pub const KEY_permille: Keysym = Keysym(0x00000ad5);
    /// minutes
    pub const KEY_minutes: Keysym = Keysym(0x00000ad6);
    /// seconds
    pub const KEY_seconds: Keysym = Keysym(0x00000ad7);
    /// latincross
    pub const KEY_latincross: Keysym = Keysym(0x00000ad9);
    /// hexagram
    pub const KEY_hexagram: Keysym = Keysym(0x00000ada);
    /// filledrectbullet
    pub const KEY_filledrectbullet: Keysym = Keysym(0x00000adb);
    /// filledlefttribullet
    pub const KEY_filledlefttribullet: Keysym = Keysym(0x00000adc);
    /// filledrighttribullet
    pub const KEY_filledrighttribullet: Keysym = Keysym(0x00000add);
    /// emfilledcircle
    pub const KEY_emfilledcircle: Keysym = Keysym(0x00000ade);
    /// emfilledrect
    pub const KEY_emfilledrect: Keysym = Keysym(0x00000adf);
    /// enopencircbullet
    pub const KEY_enopencircbullet: Keysym = Keysym(0x00000ae0);
    /// enopensquarebullet
    pub const KEY_enopensquarebullet: Keysym = Keysym(0x00000ae1);
    /// openrectbullet
    pub const KEY_openrectbullet: Keysym = Keysym(0x00000ae2);
    /// opentribulletup
    pub const KEY_opentribulletup: Keysym = Keysym(0x00000ae3);
    /// opentribulletdown
    pub const KEY_opentribulletdown: Keysym = Keysym(0x00000ae4);
    /// openstar
    pub const KEY_openstar: Keysym = Keysym(0x00000ae5);
    /// enfilledcircbullet
    pub const KEY_enfilledcircbullet: Keysym = Keysym(0x00000ae6);
    /// enfilledsqbullet
    pub const KEY_enfilledsqbullet: Keysym = Keysym(0x00000ae7);
    /// filledtribulletup
    pub const KEY_filledtribulletup: Keysym = Keysym(0x00000ae8);
    /// filledtribulletdown
    pub const KEY_filledtribulletdown: Keysym = Keysym(0x00000ae9);
    /// leftpointer
    pub const KEY_leftpointer: Keysym = Keysym(0x00000aea);
    /// rightpointer
    pub const KEY_rightpointer: Keysym = Keysym(0x00000aeb);
    /// club
    pub const KEY_club: Keysym = Keysym(0x00000aec);
    /// diamond
    pub const KEY_diamond: Keysym = Keysym(0x00000aed);
    /// heart
    pub const KEY_heart: Keysym = Keysym(0x00000aee);
    /// maltesecross
    pub const KEY_maltesecross: Keysym = Keysym(0x00000af0);
    /// dagger
    pub const KEY_dagger: Keysym = Keysym(0x00000af1);
    /// doubledagger
    pub const KEY_doubledagger: Keysym = Keysym(0x00000af2);
    /// checkmark
    pub const KEY_checkmark: Keysym = Keysym(0x00000af3);
    /// ballotcross
    pub const KEY_ballotcross: Keysym = Keysym(0x00000af4);
    /// musicalsharp
    pub const KEY_musicalsharp: Keysym = Keysym(0x00000af5);
    /// musicalflat
    pub const KEY_musicalflat: Keysym = Keysym(0x00000af6);
    /// malesymbol
    pub const KEY_malesymbol: Keysym = Keysym(0x00000af7);
    /// femalesymbol
    pub const KEY_femalesymbol: Keysym = Keysym(0x00000af8);
    /// telephone
    pub const KEY_telephone: Keysym = Keysym(0x00000af9);
    /// telephonerecorder
    pub const KEY_telephonerecorder: Keysym = Keysym(0x00000afa);
    /// phonographcopyright
    pub const KEY_phonographcopyright: Keysym = Keysym(0x00000afb);
    /// caret
    pub const KEY_caret: Keysym = Keysym(0x00000afc);
    /// singlelowquotemark
    pub const KEY_singlelowquotemark: Keysym = Keysym(0x00000afd);
    /// doublelowquotemark
    pub const KEY_doublelowquotemark: Keysym = Keysym(0x00000afe);
    /// cursor
    pub const KEY_cursor: Keysym = Keysym(0x00000aff);
    /// leftcaret
    pub const KEY_leftcaret: Keysym = Keysym(0x00000ba3);
    /// rightcaret
    pub const KEY_rightcaret: Keysym = Keysym(0x00000ba6);
    /// downcaret
    pub const KEY_downcaret: Keysym = Keysym(0x00000ba8);
    /// upcaret
    pub const KEY_upcaret: Keysym = Keysym(0x00000ba9);
    /// overbar
    pub const KEY_overbar: Keysym = Keysym(0x00000bc0);
    /// downtack
    pub const KEY_downtack: Keysym = Keysym(0x00000bc2);
    /// upshoe
    pub const KEY_upshoe: Keysym = Keysym(0x00000bc3);
    /// downstile
    pub const KEY_downstile: Keysym = Keysym(0x00000bc4);
    /// underbar
    pub const KEY_underbar: Keysym = Keysym(0x00000bc6);
    /// jot
    pub const KEY_jot: Keysym = Keysym(0x00000bca);
    /// quad
    pub const KEY_quad: Keysym = Keysym(0x00000bcc);
    /// uptack
    pub const KEY_uptack: Keysym = Keysym(0x00000bce);
    /// circle
    pub const KEY_circle: Keysym = Keysym(0x00000bcf);
    /// upstile
    pub const KEY_upstile: Keysym = Keysym(0x00000bd3);
    /// downshoe
    pub const KEY_downshoe: Keysym = Keysym(0x00000bd6);
    /// rightshoe
    pub const KEY_rightshoe: Keysym = Keysym(0x00000bd8);
    /// leftshoe
    pub const KEY_leftshoe: Keysym = Keysym(0x00000bda);
    /// lefttack
    pub const KEY_lefttack: Keysym = Keysym(0x00000bdc);
    /// righttack
    pub const KEY_righttack: Keysym = Keysym(0x00000bfc);
    /// hebrew_doublelowline
    pub const KEY_hebrew_doublelowline: Keysym = Keysym(0x00000cdf);
    /// hebrew_aleph
    pub const KEY_hebrew_aleph: Keysym = Keysym(0x00000ce0);
    /// hebrew_bet
    pub const KEY_hebrew_bet: Keysym = Keysym(0x00000ce1);
    /// hebrew_beth
    pub const KEY_hebrew_beth: Keysym = Keysym(0x00000ce1);
    /// hebrew_gimel
    pub const KEY_hebrew_gimel: Keysym = Keysym(0x00000ce2);
    /// hebrew_gimmel
    pub const KEY_hebrew_gimmel: Keysym = Keysym(0x00000ce2);
    /// hebrew_dalet
    pub const KEY_hebrew_dalet: Keysym = Keysym(0x00000ce3);
    /// hebrew_daleth
    pub const KEY_hebrew_daleth: Keysym = Keysym(0x00000ce3);
    /// hebrew_he
    pub const KEY_hebrew_he: Keysym = Keysym(0x00000ce4);
    /// hebrew_waw
    pub const KEY_hebrew_waw: Keysym = Keysym(0x00000ce5);
    /// hebrew_zain
    pub const KEY_hebrew_zain: Keysym = Keysym(0x00000ce6);
    /// hebrew_zayin
    pub const KEY_hebrew_zayin: Keysym = Keysym(0x00000ce6);
    /// hebrew_chet
    pub const KEY_hebrew_chet: Keysym = Keysym(0x00000ce7);
    /// hebrew_het
    pub const KEY_hebrew_het: Keysym = Keysym(0x00000ce7);
    /// hebrew_tet
    pub const KEY_hebrew_tet: Keysym = Keysym(0x00000ce8);
    /// hebrew_teth
    pub const KEY_hebrew_teth: Keysym = Keysym(0x00000ce8);
    /// hebrew_yod
    pub const KEY_hebrew_yod: Keysym = Keysym(0x00000ce9);
    /// hebrew_finalkaph
    pub const KEY_hebrew_finalkaph: Keysym = Keysym(0x00000cea);
    /// hebrew_kaph
    pub const KEY_hebrew_kaph: Keysym = Keysym(0x00000ceb);
    /// hebrew_lamed
    pub const KEY_hebrew_lamed: Keysym = Keysym(0x00000cec);
    /// hebrew_finalmem
    pub const KEY_hebrew_finalmem: Keysym = Keysym(0x00000ced);
    /// hebrew_mem
    pub const KEY_hebrew_mem: Keysym = Keysym(0x00000cee);
    /// hebrew_finalnun
    pub const KEY_hebrew_finalnun: Keysym = Keysym(0x00000cef);
    /// hebrew_nun
    pub const KEY_hebrew_nun: Keysym = Keysym(0x00000cf0);
    /// hebrew_samech
    pub const KEY_hebrew_samech: Keysym = Keysym(0x00000cf1);
    /// hebrew_samekh
    pub const KEY_hebrew_samekh: Keysym = Keysym(0x00000cf1);
    /// hebrew_ayin
    pub const KEY_hebrew_ayin: Keysym = Keysym(0x00000cf2);
    /// hebrew_finalpe
    pub const KEY_hebrew_finalpe: Keysym = Keysym(0x00000cf3);
    /// hebrew_pe
    pub const KEY_hebrew_pe: Keysym = Keysym(0x00000cf4);
    /// hebrew_finalzade
    pub const KEY_hebrew_finalzade: Keysym = Keysym(0x00000cf5);
    /// hebrew_finalzadi
    pub const KEY_hebrew_finalzadi: Keysym = Keysym(0x00000cf5);
    /// hebrew_zade
    pub const KEY_hebrew_zade: Keysym = Keysym(0x00000cf6);
    /// hebrew_zadi
    pub const KEY_hebrew_zadi: Keysym = Keysym(0x00000cf6);
    /// hebrew_qoph
    pub const KEY_hebrew_qoph: Keysym = Keysym(0x00000cf7);
    /// hebrew_kuf
    pub const KEY_hebrew_kuf: Keysym = Keysym(0x00000cf7);
    /// hebrew_resh
    pub const KEY_hebrew_resh: Keysym = Keysym(0x00000cf8);
    /// hebrew_shin
    pub const KEY_hebrew_shin: Keysym = Keysym(0x00000cf9);
    /// hebrew_taw
    pub const KEY_hebrew_taw: Keysym = Keysym(0x00000cfa);
    /// hebrew_taf
    pub const KEY_hebrew_taf: Keysym = Keysym(0x00000cfa);
    /// Thai_kokai
    pub const KEY_Thai_kokai: Keysym = Keysym(0x00000da1);
    /// Thai_khokhai
    pub const KEY_Thai_khokhai: Keysym = Keysym(0x00000da2);
    /// Thai_khokhuat
    pub const KEY_Thai_khokhuat: Keysym = Keysym(0x00000da3);
    /// Thai_khokhwai
    pub const KEY_Thai_khokhwai: Keysym = Keysym(0x00000da4);
    /// Thai_khokhon
    pub const KEY_Thai_khokhon: Keysym = Keysym(0x00000da5);
    /// Thai_khorakhang
    pub const KEY_Thai_khorakhang: Keysym = Keysym(0x00000da6);
    /// Thai_ngongu
    pub const KEY_Thai_ngongu: Keysym = Keysym(0x00000da7);
    /// Thai_chochan
    pub const KEY_Thai_chochan: Keysym = Keysym(0x00000da8);
    /// Thai_choching
    pub const KEY_Thai_choching: Keysym = Keysym(0x00000da9);
    /// Thai_chochang
    pub const KEY_Thai_chochang: Keysym = Keysym(0x00000daa);
    /// Thai_soso
    pub const KEY_Thai_soso: Keysym = Keysym(0x00000dab);
    /// Thai_chochoe
    pub const KEY_Thai_chochoe: Keysym = Keysym(0x00000dac);
    /// Thai_yoying
    pub const KEY_Thai_yoying: Keysym = Keysym(0x00000dad);
    /// Thai_dochada
    pub const KEY_Thai_dochada: Keysym = Keysym(0x00000dae);
    /// Thai_topatak
    pub const KEY_Thai_topatak: Keysym = Keysym(0x00000daf);
    /// Thai_thothan
    pub const KEY_Thai_thothan: Keysym = Keysym(0x00000db0);
    /// Thai_thonangmontho
    pub const KEY_Thai_thonangmontho: Keysym = Keysym(0x00000db1);
    /// Thai_thophuthao
    pub const KEY_Thai_thophuthao: Keysym = Keysym(0x00000db2);
    /// Thai_nonen
    pub const KEY_Thai_nonen: Keysym = Keysym(0x00000db3);
    /// Thai_dodek
    pub const KEY_Thai_dodek: Keysym = Keysym(0x00000db4);
    /// Thai_totao
    pub const KEY_Thai_totao: Keysym = Keysym(0x00000db5);
    /// Thai_thothung
    pub const KEY_Thai_thothung: Keysym = Keysym(0x00000db6);
    /// Thai_thothahan
    pub const KEY_Thai_thothahan: Keysym = Keysym(0x00000db7);
    /// Thai_thothong
    pub const KEY_Thai_thothong: Keysym = Keysym(0x00000db8);
    /// Thai_nonu
    pub const KEY_Thai_nonu: Keysym = Keysym(0x00000db9);
    /// Thai_bobaimai
    pub const KEY_Thai_bobaimai: Keysym = Keysym(0x00000dba);
    /// Thai_popla
    pub const KEY_Thai_popla: Keysym = Keysym(0x00000dbb);
    /// Thai_phophung
    pub const KEY_Thai_phophung: Keysym = Keysym(0x00000dbc);
    /// Thai_fofa
    pub const KEY_Thai_fofa: Keysym = Keysym(0x00000dbd);
    /// Thai_phophan
    pub const KEY_Thai_phophan: Keysym = Keysym(0x00000dbe);
    /// Thai_fofan
    pub const KEY_Thai_fofan: Keysym = Keysym(0x00000dbf);
    /// Thai_phosamphao
    pub const KEY_Thai_phosamphao: Keysym = Keysym(0x00000dc0);
    /// Thai_moma
    pub const KEY_Thai_moma: Keysym = Keysym(0x00000dc1);
    /// Thai_yoyak
    pub const KEY_Thai_yoyak: Keysym = Keysym(0x00000dc2);
    /// Thai_rorua
    pub const KEY_Thai_rorua: Keysym = Keysym(0x00000dc3);
    /// Thai_ru
    pub const KEY_Thai_ru: Keysym = Keysym(0x00000dc4);
    /// Thai_loling
    pub const KEY_Thai_loling: Keysym = Keysym(0x00000dc5);
    /// Thai_lu
    pub const KEY_Thai_lu: Keysym = Keysym(0x00000dc6);
    /// Thai_wowaen
    pub const KEY_Thai_wowaen: Keysym = Keysym(0x00000dc7);
    /// Thai_sosala
    pub const KEY_Thai_sosala: Keysym = Keysym(0x00000dc8);
    /// Thai_sorusi
    pub const KEY_Thai_sorusi: Keysym = Keysym(0x00000dc9);
    /// Thai_sosua
    pub const KEY_Thai_sosua: Keysym = Keysym(0x00000dca);
    /// Thai_hohip
    pub const KEY_Thai_hohip: Keysym = Keysym(0x00000dcb);
    /// Thai_lochula
    pub const KEY_Thai_lochula: Keysym = Keysym(0x00000dcc);
    /// Thai_oang
    pub const KEY_Thai_oang: Keysym = Keysym(0x00000dcd);
    /// Thai_honokhuk
    pub const KEY_Thai_honokhuk: Keysym = Keysym(0x00000dce);
    /// Thai_paiyannoi
    pub const KEY_Thai_paiyannoi: Keysym = Keysym(0x00000dcf);
    /// Thai_saraa
    pub const KEY_Thai_saraa: Keysym = Keysym(0x00000dd0);
    /// Thai_maihanakat
    pub const KEY_Thai_maihanakat: Keysym = Keysym(0x00000dd1);
    /// Thai_saraaa
    pub const KEY_Thai_saraaa: Keysym = Keysym(0x00000dd2);
    /// Thai_saraam
    pub const KEY_Thai_saraam: Keysym = Keysym(0x00000dd3);
    /// Thai_sarai
    pub const KEY_Thai_sarai: Keysym = Keysym(0x00000dd4);
    /// Thai_saraii
    pub const KEY_Thai_saraii: Keysym = Keysym(0x00000dd5);
    /// Thai_saraue
    pub const KEY_Thai_saraue: Keysym = Keysym(0x00000dd6);
    /// Thai_sarauee
    pub const KEY_Thai_sarauee: Keysym = Keysym(0x00000dd7);
    /// Thai_sarau
    pub const KEY_Thai_sarau: Keysym = Keysym(0x00000dd8);
    /// Thai_sarauu
    pub const KEY_Thai_sarauu: Keysym = Keysym(0x00000dd9);
    /// Thai_phinthu
    pub const KEY_Thai_phinthu: Keysym = Keysym(0x00000dda);
    /// Thai_maihanakat_maitho
    pub const KEY_Thai_maihanakat_maitho: Keysym = Keysym(0x00000dde);
    /// Thai_baht
    pub const KEY_Thai_baht: Keysym = Keysym(0x00000ddf);
    /// Thai_sarae
    pub const KEY_Thai_sarae: Keysym = Keysym(0x00000de0);
    /// Thai_saraae
    pub const KEY_Thai_saraae: Keysym = Keysym(0x00000de1);
    /// Thai_sarao
    pub const KEY_Thai_sarao: Keysym = Keysym(0x00000de2);
    /// Thai_saraaimaimuan
    pub const KEY_Thai_saraaimaimuan: Keysym = Keysym(0x00000de3);
    /// Thai_saraaimaimalai
    pub const KEY_Thai_saraaimaimalai: Keysym = Keysym(0x00000de4);
    /// Thai_lakkhangyao
    pub const KEY_Thai_lakkhangyao: Keysym = Keysym(0x00000de5);
    /// Thai_maiyamok
    pub const KEY_Thai_maiyamok: Keysym = Keysym(0x00000de6);
    /// Thai_maitaikhu
    pub const KEY_Thai_maitaikhu: Keysym = Keysym(0x00000de7);
    /// Thai_maiek
    pub const KEY_Thai_maiek: Keysym = Keysym(0x00000de8);
    /// Thai_maitho
    pub const KEY_Thai_maitho: Keysym = Keysym(0x00000de9);
    /// Thai_maitri
    pub const KEY_Thai_maitri: Keysym = Keysym(0x00000dea);
    /// Thai_maichattawa
    pub const KEY_Thai_maichattawa: Keysym = Keysym(0x00000deb);
    /// Thai_thanthakhat
    pub const KEY_Thai_thanthakhat: Keysym = Keysym(0x00000dec);
    /// Thai_nikhahit
    pub const KEY_Thai_nikhahit: Keysym = Keysym(0x00000ded);
    /// Thai_leksun
    pub const KEY_Thai_leksun: Keysym = Keysym(0x00000df0);
    /// Thai_leknung
    pub const KEY_Thai_leknung: Keysym = Keysym(0x00000df1);
    /// Thai_leksong
    pub const KEY_Thai_leksong: Keysym = Keysym(0x00000df2);
    /// Thai_leksam
    pub const KEY_Thai_leksam: Keysym = Keysym(0x00000df3);
    /// Thai_leksi
    pub const KEY_Thai_leksi: Keysym = Keysym(0x00000df4);
    /// Thai_lekha
    pub const KEY_Thai_lekha: Keysym = Keysym(0x00000df5);
    /// Thai_lekhok
    pub const KEY_Thai_lekhok: Keysym = Keysym(0x00000df6);
    /// Thai_lekchet
    pub const KEY_Thai_lekchet: Keysym = Keysym(0x00000df7);
    /// Thai_lekpaet
    pub const KEY_Thai_lekpaet: Keysym = Keysym(0x00000df8);
    /// Thai_lekkao
    pub const KEY_Thai_lekkao: Keysym = Keysym(0x00000df9);
    /// Hangul
    pub const KEY_Hangul: Keysym = Keysym(0x0000ff31);
    /// Hangul_Start
    pub const KEY_Hangul_Start: Keysym = Keysym(0x0000ff32);
    /// Hangul_End
    pub const KEY_Hangul_End: Keysym = Keysym(0x0000ff33);
    /// Hangul_Hanja
    pub const KEY_Hangul_Hanja: Keysym = Keysym(0x0000ff34);
    /// Hangul_Jamo
    pub const KEY_Hangul_Jamo: Keysym = Keysym(0x0000ff35);
    /// Hangul_Romaja
    pub const KEY_Hangul_Romaja: Keysym = Keysym(0x0000ff36);
    /// Hangul_Jeonja
    pub const KEY_Hangul_Jeonja: Keysym = Keysym(0x0000ff38);
    /// Hangul_Banja
    pub const KEY_Hangul_Banja: Keysym = Keysym(0x0000ff39);
    /// Hangul_PreHanja
    pub const KEY_Hangul_PreHanja: Keysym = Keysym(0x0000ff3a);
    /// Hangul_PostHanja
    pub const KEY_Hangul_PostHanja: Keysym = Keysym(0x0000ff3b);
    /// Hangul_Special
    pub const KEY_Hangul_Special: Keysym = Keysym(0x0000ff3f);
    /// Hangul_Kiyeog
    pub const KEY_Hangul_Kiyeog: Keysym = Keysym(0x00000ea1);
    /// Hangul_SsangKiyeog
    pub const KEY_Hangul_SsangKiyeog: Keysym = Keysym(0x00000ea2);
    /// Hangul_KiyeogSios
    pub const KEY_Hangul_KiyeogSios: Keysym = Keysym(0x00000ea3);
    /// Hangul_Nieun
    pub const KEY_Hangul_Nieun: Keysym = Keysym(0x00000ea4);
    /// Hangul_NieunJieuj
    pub const KEY_Hangul_NieunJieuj: Keysym = Keysym(0x00000ea5);
    /// Hangul_NieunHieuh
    pub const KEY_Hangul_NieunHieuh: Keysym = Keysym(0x00000ea6);
    /// Hangul_Dikeud
    pub const KEY_Hangul_Dikeud: Keysym = Keysym(0x00000ea7);
    /// Hangul_SsangDikeud
    pub const KEY_Hangul_SsangDikeud: Keysym = Keysym(0x00000ea8);
    /// Hangul_Rieul
    pub const KEY_Hangul_Rieul: Keysym = Keysym(0x00000ea9);
    /// Hangul_RieulKiyeog
    pub const KEY_Hangul_RieulKiyeog: Keysym = Keysym(0x00000eaa);
    /// Hangul_RieulMieum
    pub const KEY_Hangul_RieulMieum: Keysym = Keysym(0x00000eab);
    /// Hangul_RieulPieub
    pub const KEY_Hangul_RieulPieub: Keysym = Keysym(0x00000eac);
    /// Hangul_RieulSios
    pub const KEY_Hangul_RieulSios: Keysym = Keysym(0x00000ead);
    /// Hangul_RieulTieut
    pub const KEY_Hangul_RieulTieut: Keysym = Keysym(0x00000eae);
    /// Hangul_RieulPhieuf
    pub const KEY_Hangul_RieulPhieuf: Keysym = Keysym(0x00000eaf);
    /// Hangul_RieulHieuh
    pub const KEY_Hangul_RieulHieuh: Keysym = Keysym(0x00000eb0);
    /// Hangul_Mieum
    pub const KEY_Hangul_Mieum: Keysym = Keysym(0x00000eb1);
    /// Hangul_Pieub
    pub const KEY_Hangul_Pieub: Keysym = Keysym(0x00000eb2);
    /// Hangul_SsangPieub
    pub const KEY_Hangul_SsangPieub: Keysym = Keysym(0x00000eb3);
    /// Hangul_PieubSios
    pub const KEY_Hangul_PieubSios: Keysym = Keysym(0x00000eb4);
    /// Hangul_Sios
    pub const KEY_Hangul_Sios: Keysym = Keysym(0x00000eb5);
    /// Hangul_SsangSios
    pub const KEY_Hangul_SsangSios: Keysym = Keysym(0x00000eb6);
    /// Hangul_Ieung
    pub const KEY_Hangul_Ieung: Keysym = Keysym(0x00000eb7);
    /// Hangul_Jieuj
    pub const KEY_Hangul_Jieuj: Keysym = Keysym(0x00000eb8);
    /// Hangul_SsangJieuj
    pub const KEY_Hangul_SsangJieuj: Keysym = Keysym(0x00000eb9);
    /// Hangul_Cieuc
    pub const KEY_Hangul_Cieuc: Keysym = Keysym(0x00000eba);
    /// Hangul_Khieuq
    pub const KEY_Hangul_Khieuq: Keysym = Keysym(0x00000ebb);
    /// Hangul_Tieut
    pub const KEY_Hangul_Tieut: Keysym = Keysym(0x00000ebc);
    /// Hangul_Phieuf
    pub const KEY_Hangul_Phieuf: Keysym = Keysym(0x00000ebd);
    /// Hangul_Hieuh
    pub const KEY_Hangul_Hieuh: Keysym = Keysym(0x00000ebe);
    /// Hangul_A
    pub const KEY_Hangul_A: Keysym = Keysym(0x00000ebf);
    /// Hangul_AE
    pub const KEY_Hangul_AE: Keysym = Keysym(0x00000ec0);
    /// Hangul_YA
    pub const KEY_Hangul_YA: Keysym = Keysym(0x00000ec1);
    /// Hangul_YAE
    pub const KEY_Hangul_YAE: Keysym = Keysym(0x00000ec2);
    /// Hangul_EO
    pub const KEY_Hangul_EO: Keysym = Keysym(0x00000ec3);
    /// Hangul_E
    pub const KEY_Hangul_E: Keysym = Keysym(0x00000ec4);
    /// Hangul_YEO
    pub const KEY_Hangul_YEO: Keysym = Keysym(0x00000ec5);
    /// Hangul_YE
    pub const KEY_Hangul_YE: Keysym = Keysym(0x00000ec6);
    /// Hangul_O
    pub const KEY_Hangul_O: Keysym = Keysym(0x00000ec7);
    /// Hangul_WA
    pub const KEY_Hangul_WA: Keysym = Keysym(0x00000ec8);
    /// Hangul_WAE
    pub const KEY_Hangul_WAE: Keysym = Keysym(0x00000ec9);
    /// Hangul_OE
    pub const KEY_Hangul_OE: Keysym = Keysym(0x00000eca);
    /// Hangul_YO
    pub const KEY_Hangul_YO: Keysym = Keysym(0x00000ecb);
    /// Hangul_U
    pub const KEY_Hangul_U: Keysym = Keysym(0x00000ecc);
    /// Hangul_WEO
    pub const KEY_Hangul_WEO: Keysym = Keysym(0x00000ecd);
    /// Hangul_WE
    pub const KEY_Hangul_WE: Keysym = Keysym(0x00000ece);
    /// Hangul_WI
    pub const KEY_Hangul_WI: Keysym = Keysym(0x00000ecf);
    /// Hangul_YU
    pub const KEY_Hangul_YU: Keysym = Keysym(0x00000ed0);
    /// Hangul_EU
    pub const KEY_Hangul_EU: Keysym = Keysym(0x00000ed1);
    /// Hangul_YI
    pub const KEY_Hangul_YI: Keysym = Keysym(0x00000ed2);
    /// Hangul_I
    pub const KEY_Hangul_I: Keysym = Keysym(0x00000ed3);
    /// Hangul_J_Kiyeog
    pub const KEY_Hangul_J_Kiyeog: Keysym = Keysym(0x00000ed4);
    /// Hangul_J_SsangKiyeog
    pub const KEY_Hangul_J_SsangKiyeog: Keysym = Keysym(0x00000ed5);
    /// Hangul_J_KiyeogSios
    pub const KEY_Hangul_J_KiyeogSios: Keysym = Keysym(0x00000ed6);
    /// Hangul_J_Nieun
    pub const KEY_Hangul_J_Nieun: Keysym = Keysym(0x00000ed7);
    /// Hangul_J_NieunJieuj
    pub const KEY_Hangul_J_NieunJieuj: Keysym = Keysym(0x00000ed8);
    /// Hangul_J_NieunHieuh
    pub const KEY_Hangul_J_NieunHieuh: Keysym = Keysym(0x00000ed9);
    /// Hangul_J_Dikeud
    pub const KEY_Hangul_J_Dikeud: Keysym = Keysym(0x00000eda);
    /// Hangul_J_Rieul
    pub const KEY_Hangul_J_Rieul: Keysym = Keysym(0x00000edb);
    /// Hangul_J_RieulKiyeog
    pub const KEY_Hangul_J_RieulKiyeog: Keysym = Keysym(0x00000edc);
    /// Hangul_J_RieulMieum
    pub const KEY_Hangul_J_RieulMieum: Keysym = Keysym(0x00000edd);
    /// Hangul_J_RieulPieub
    pub const KEY_Hangul_J_RieulPieub: Keysym = Keysym(0x00000ede);
    /// Hangul_J_RieulSios
    pub const KEY_Hangul_J_RieulSios: Keysym = Keysym(0x00000edf);
    /// Hangul_J_RieulTieut
    pub const KEY_Hangul_J_RieulTieut: Keysym = Keysym(0x00000ee0);
    /// Hangul_J_RieulPhieuf
    pub const KEY_Hangul_J_RieulPhieuf: Keysym = Keysym(0x00000ee1);
    /// Hangul_J_RieulHieuh
    pub const KEY_Hangul_J_RieulHieuh: Keysym = Keysym(0x00000ee2);
    /// Hangul_J_Mieum
    pub const KEY_Hangul_J_Mieum: Keysym = Keysym(0x00000ee3);
    /// Hangul_J_Pieub
    pub const KEY_Hangul_J_Pieub: Keysym = Keysym(0x00000ee4);
    /// Hangul_J_PieubSios
    pub const KEY_Hangul_J_PieubSios: Keysym = Keysym(0x00000ee5);
    /// Hangul_J_Sios
    pub const KEY_Hangul_J_Sios: Keysym = Keysym(0x00000ee6);
    /// Hangul_J_SsangSios
    pub const KEY_Hangul_J_SsangSios: Keysym = Keysym(0x00000ee7);
    /// Hangul_J_Ieung
    pub const KEY_Hangul_J_Ieung: Keysym = Keysym(0x00000ee8);
    /// Hangul_J_Jieuj
    pub const KEY_Hangul_J_Jieuj: Keysym = Keysym(0x00000ee9);
    /// Hangul_J_Cieuc
    pub const KEY_Hangul_J_Cieuc: Keysym = Keysym(0x00000eea);
    /// Hangul_J_Khieuq
    pub const KEY_Hangul_J_Khieuq: Keysym = Keysym(0x00000eeb);
    /// Hangul_J_Tieut
    pub const KEY_Hangul_J_Tieut: Keysym = Keysym(0x00000eec);
    /// Hangul_J_Phieuf
    pub const KEY_Hangul_J_Phieuf: Keysym = Keysym(0x00000eed);
    /// Hangul_J_Hieuh
    pub const KEY_Hangul_J_Hieuh: Keysym = Keysym(0x00000eee);
    /// Hangul_RieulYeorinHieuh
    pub const KEY_Hangul_RieulYeorinHieuh: Keysym = Keysym(0x00000eef);
    /// Hangul_SunkyeongeumMieum
    pub const KEY_Hangul_SunkyeongeumMieum: Keysym = Keysym(0x00000ef0);
    /// Hangul_SunkyeongeumPieub
    pub const KEY_Hangul_SunkyeongeumPieub: Keysym = Keysym(0x00000ef1);
    /// Hangul_PanSios
    pub const KEY_Hangul_PanSios: Keysym = Keysym(0x00000ef2);
    /// Hangul_KkogjiDalrinIeung
    pub const KEY_Hangul_KkogjiDalrinIeung: Keysym = Keysym(0x00000ef3);
    /// Hangul_SunkyeongeumPhieuf
    pub const KEY_Hangul_SunkyeongeumPhieuf: Keysym = Keysym(0x00000ef4);
    /// Hangul_YeorinHieuh
    pub const KEY_Hangul_YeorinHieuh: Keysym = Keysym(0x00000ef5);
    /// Hangul_AraeA
    pub const KEY_Hangul_AraeA: Keysym = Keysym(0x00000ef6);
    /// Hangul_AraeAE
    pub const KEY_Hangul_AraeAE: Keysym = Keysym(0x00000ef7);
    /// Hangul_J_PanSios
    pub const KEY_Hangul_J_PanSios: Keysym = Keysym(0x00000ef8);
    /// Hangul_J_KkogjiDalrinIeung
    pub const KEY_Hangul_J_KkogjiDalrinIeung: Keysym = Keysym(0x00000ef9);
    /// Hangul_J_YeorinHieuh
    pub const KEY_Hangul_J_YeorinHieuh: Keysym = Keysym(0x00000efa);
    /// Korean_Won
    pub const KEY_Korean_Won: Keysym = Keysym(0x00000eff);
    /// Armenian_ligature_ew
    pub const KEY_Armenian_ligature_ew: Keysym = Keysym(0x01000587);
    /// Armenian_full_stop
    pub const KEY_Armenian_full_stop: Keysym = Keysym(0x01000589);
    /// Armenian_verjaket
    pub const KEY_Armenian_verjaket: Keysym = Keysym(0x01000589);
    /// Armenian_separation_mark
    pub const KEY_Armenian_separation_mark: Keysym = Keysym(0x0100055d);
    /// Armenian_but
    pub const KEY_Armenian_but: Keysym = Keysym(0x0100055d);
    /// Armenian_hyphen
    pub const KEY_Armenian_hyphen: Keysym = Keysym(0x0100058a);
    /// Armenian_yentamna
    pub const KEY_Armenian_yentamna: Keysym = Keysym(0x0100058a);
    /// Armenian_exclam
    pub const KEY_Armenian_exclam: Keysym = Keysym(0x0100055c);
    /// Armenian_amanak
    pub const KEY_Armenian_amanak: Keysym = Keysym(0x0100055c);
    /// Armenian_accent
    pub const KEY_Armenian_accent: Keysym = Keysym(0x0100055b);
    /// Armenian_shesht
    pub const KEY_Armenian_shesht: Keysym = Keysym(0x0100055b);
    /// Armenian_question
    pub const KEY_Armenian_question: Keysym = Keysym(0x0100055e);
    /// Armenian_paruyk
    pub const KEY_Armenian_paruyk: Keysym = Keysym(0x0100055e);
    /// Armenian_AYB
    pub const KEY_Armenian_AYB: Keysym = Keysym(0x01000531);
    /// Armenian_ayb
    pub const KEY_Armenian_ayb: Keysym = Keysym(0x01000561);
    /// Armenian_BEN
    pub const KEY_Armenian_BEN: Keysym = Keysym(0x01000532);
    /// Armenian_ben
    pub const KEY_Armenian_ben: Keysym = Keysym(0x01000562);
    /// Armenian_GIM
    pub const KEY_Armenian_GIM: Keysym = Keysym(0x01000533);
    /// Armenian_gim
    pub const KEY_Armenian_gim: Keysym = Keysym(0x01000563);
    /// Armenian_DA
    pub const KEY_Armenian_DA: Keysym = Keysym(0x01000534);
    /// Armenian_da
    pub const KEY_Armenian_da: Keysym = Keysym(0x01000564);
    /// Armenian_YECH
    pub const KEY_Armenian_YECH: Keysym = Keysym(0x01000535);
    /// Armenian_yech
    pub const KEY_Armenian_yech: Keysym = Keysym(0x01000565);
    /// Armenian_ZA
    pub const KEY_Armenian_ZA: Keysym = Keysym(0x01000536);
    /// Armenian_za
    pub const KEY_Armenian_za: Keysym = Keysym(0x01000566);
    /// Armenian_E
    pub const KEY_Armenian_E: Keysym = Keysym(0x01000537);
    /// Armenian_e
    pub const KEY_Armenian_e: Keysym = Keysym(0x01000567);
    /// Armenian_AT
    pub const KEY_Armenian_AT: Keysym = Keysym(0x01000538);
    /// Armenian_at
    pub const KEY_Armenian_at: Keysym = Keysym(0x01000568);
    /// Armenian_TO
    pub const KEY_Armenian_TO: Keysym = Keysym(0x01000539);
    /// Armenian_to
    pub const KEY_Armenian_to: Keysym = Keysym(0x01000569);
    /// Armenian_ZHE
    pub const KEY_Armenian_ZHE: Keysym = Keysym(0x0100053a);
    /// Armenian_zhe
    pub const KEY_Armenian_zhe: Keysym = Keysym(0x0100056a);
    /// Armenian_INI
    pub const KEY_Armenian_INI: Keysym = Keysym(0x0100053b);
    /// Armenian_ini
    pub const KEY_Armenian_ini: Keysym = Keysym(0x0100056b);
    /// Armenian_LYUN
    pub const KEY_Armenian_LYUN: Keysym = Keysym(0x0100053c);
    /// Armenian_lyun
    pub const KEY_Armenian_lyun: Keysym = Keysym(0x0100056c);
    /// Armenian_KHE
    pub const KEY_Armenian_KHE: Keysym = Keysym(0x0100053d);
    /// Armenian_khe
    pub const KEY_Armenian_khe: Keysym = Keysym(0x0100056d);
    /// Armenian_TSA
    pub const KEY_Armenian_TSA: Keysym = Keysym(0x0100053e);
    /// Armenian_tsa
    pub const KEY_Armenian_tsa: Keysym = Keysym(0x0100056e);
    /// Armenian_KEN
    pub const KEY_Armenian_KEN: Keysym = Keysym(0x0100053f);
    /// Armenian_ken
    pub const KEY_Armenian_ken: Keysym = Keysym(0x0100056f);
    /// Armenian_HO
    pub const KEY_Armenian_HO: Keysym = Keysym(0x01000540);
    /// Armenian_ho
    pub const KEY_Armenian_ho: Keysym = Keysym(0x01000570);
    /// Armenian_DZA
    pub const KEY_Armenian_DZA: Keysym = Keysym(0x01000541);
    /// Armenian_dza
    pub const KEY_Armenian_dza: Keysym = Keysym(0x01000571);
    /// Armenian_GHAT
    pub const KEY_Armenian_GHAT: Keysym = Keysym(0x01000542);
    /// Armenian_ghat
    pub const KEY_Armenian_ghat: Keysym = Keysym(0x01000572);
    /// Armenian_TCHE
    pub const KEY_Armenian_TCHE: Keysym = Keysym(0x01000543);
    /// Armenian_tche
    pub const KEY_Armenian_tche: Keysym = Keysym(0x01000573);
    /// Armenian_MEN
    pub const KEY_Armenian_MEN: Keysym = Keysym(0x01000544);
    /// Armenian_men
    pub const KEY_Armenian_men: Keysym = Keysym(0x01000574);
    /// Armenian_HI
    pub const KEY_Armenian_HI: Keysym = Keysym(0x01000545);
    /// Armenian_hi
    pub const KEY_Armenian_hi: Keysym = Keysym(0x01000575);
    /// Armenian_NU
    pub const KEY_Armenian_NU: Keysym = Keysym(0x01000546);
    /// Armenian_nu
    pub const KEY_Armenian_nu: Keysym = Keysym(0x01000576);
    /// Armenian_SHA
    pub const KEY_Armenian_SHA: Keysym = Keysym(0x01000547);
    /// Armenian_sha
    pub const KEY_Armenian_sha: Keysym = Keysym(0x01000577);
    /// Armenian_VO
    pub const KEY_Armenian_VO: Keysym = Keysym(0x01000548);
    /// Armenian_vo
    pub const KEY_Armenian_vo: Keysym = Keysym(0x01000578);
    /// Armenian_CHA
    pub const KEY_Armenian_CHA: Keysym = Keysym(0x01000549);
    /// Armenian_cha
    pub const KEY_Armenian_cha: Keysym = Keysym(0x01000579);
    /// Armenian_PE
    pub const KEY_Armenian_PE: Keysym = Keysym(0x0100054a);
    /// Armenian_pe
    pub const KEY_Armenian_pe: Keysym = Keysym(0x0100057a);
    /// Armenian_JE
    pub const KEY_Armenian_JE: Keysym = Keysym(0x0100054b);
    /// Armenian_je
    pub const KEY_Armenian_je: Keysym = Keysym(0x0100057b);
    /// Armenian_RA
    pub const KEY_Armenian_RA: Keysym = Keysym(0x0100054c);
    /// Armenian_ra
    pub const KEY_Armenian_ra: Keysym = Keysym(0x0100057c);
    /// Armenian_SE
    pub const KEY_Armenian_SE: Keysym = Keysym(0x0100054d);
    /// Armenian_se
    pub const KEY_Armenian_se: Keysym = Keysym(0x0100057d);
    /// Armenian_VEV
    pub const KEY_Armenian_VEV: Keysym = Keysym(0x0100054e);
    /// Armenian_vev
    pub const KEY_Armenian_vev: Keysym = Keysym(0x0100057e);
    /// Armenian_TYUN
    pub const KEY_Armenian_TYUN: Keysym = Keysym(0x0100054f);
    /// Armenian_tyun
    pub const KEY_Armenian_tyun: Keysym = Keysym(0x0100057f);
    /// Armenian_RE
    pub const KEY_Armenian_RE: Keysym = Keysym(0x01000550);
    /// Armenian_re
    pub const KEY_Armenian_re: Keysym = Keysym(0x01000580);
    /// Armenian_TSO
    pub const KEY_Armenian_TSO: Keysym = Keysym(0x01000551);
    /// Armenian_tso
    pub const KEY_Armenian_tso: Keysym = Keysym(0x01000581);
    /// Armenian_VYUN
    pub const KEY_Armenian_VYUN: Keysym = Keysym(0x01000552);
    /// Armenian_vyun
    pub const KEY_Armenian_vyun: Keysym = Keysym(0x01000582);
    /// Armenian_PYUR
    pub const KEY_Armenian_PYUR: Keysym = Keysym(0x01000553);
    /// Armenian_pyur
    pub const KEY_Armenian_pyur: Keysym = Keysym(0x01000583);
    /// Armenian_KE
    pub const KEY_Armenian_KE: Keysym = Keysym(0x01000554);
    /// Armenian_ke
    pub const KEY_Armenian_ke: Keysym = Keysym(0x01000584);
    /// Armenian_O
    pub const KEY_Armenian_O: Keysym = Keysym(0x01000555);
    /// Armenian_o
    pub const KEY_Armenian_o: Keysym = Keysym(0x01000585);
    /// Armenian_FE
    pub const KEY_Armenian_FE: Keysym = Keysym(0x01000556);
    /// Armenian_fe
    pub const KEY_Armenian_fe: Keysym = Keysym(0x01000586);
    /// Armenian_apostrophe
    pub const KEY_Armenian_apostrophe: Keysym = Keysym(0x0100055a);
    /// Georgian_an
    pub const KEY_Georgian_an: Keysym = Keysym(0x010010d0);
    /// Georgian_ban
    pub const KEY_Georgian_ban: Keysym = Keysym(0x010010d1);
    /// Georgian_gan
    pub const KEY_Georgian_gan: Keysym = Keysym(0x010010d2);
    /// Georgian_don
    pub const KEY_Georgian_don: Keysym = Keysym(0x010010d3);
    /// Georgian_en
    pub const KEY_Georgian_en: Keysym = Keysym(0x010010d4);
    /// Georgian_vin
    pub const KEY_Georgian_vin: Keysym = Keysym(0x010010d5);
    /// Georgian_zen
    pub const KEY_Georgian_zen: Keysym = Keysym(0x010010d6);
    /// Georgian_tan
    pub const KEY_Georgian_tan: Keysym = Keysym(0x010010d7);
    /// Georgian_in
    pub const KEY_Georgian_in: Keysym = Keysym(0x010010d8);
    /// Georgian_kan
    pub const KEY_Georgian_kan: Keysym = Keysym(0x010010d9);
    /// Georgian_las
    pub const KEY_Georgian_las: Keysym = Keysym(0x010010da);
    /// Georgian_man
    pub const KEY_Georgian_man: Keysym = Keysym(0x010010db);
    /// Georgian_nar
    pub const KEY_Georgian_nar: Keysym = Keysym(0x010010dc);
    /// Georgian_on
    pub const KEY_Georgian_on: Keysym = Keysym(0x010010dd);
    /// Georgian_par
    pub const KEY_Georgian_par: Keysym = Keysym(0x010010de);
    /// Georgian_zhar
    pub const KEY_Georgian_zhar: Keysym = Keysym(0x010010df);
    /// Georgian_rae
    pub const KEY_Georgian_rae: Keysym = Keysym(0x010010e0);
    /// Georgian_san
    pub const KEY_Georgian_san: Keysym = Keysym(0x010010e1);
    /// Georgian_tar
    pub const KEY_Georgian_tar: Keysym = Keysym(0x010010e2);
    /// Georgian_un
    pub const KEY_Georgian_un: Keysym = Keysym(0x010010e3);
    /// Georgian_phar
    pub const KEY_Georgian_phar: Keysym = Keysym(0x010010e4);
    /// Georgian_khar
    pub const KEY_Georgian_khar: Keysym = Keysym(0x010010e5);
    /// Georgian_ghan
    pub const KEY_Georgian_ghan: Keysym = Keysym(0x010010e6);
    /// Georgian_qar
    pub const KEY_Georgian_qar: Keysym = Keysym(0x010010e7);
    /// Georgian_shin
    pub const KEY_Georgian_shin: Keysym = Keysym(0x010010e8);
    /// Georgian_chin
    pub const KEY_Georgian_chin: Keysym = Keysym(0x010010e9);
    /// Georgian_can
    pub const KEY_Georgian_can: Keysym = Keysym(0x010010ea);
    /// Georgian_jil
    pub const KEY_Georgian_jil: Keysym = Keysym(0x010010eb);
    /// Georgian_cil
    pub const KEY_Georgian_cil: Keysym = Keysym(0x010010ec);
    /// Georgian_char
    pub const KEY_Georgian_char: Keysym = Keysym(0x010010ed);
    /// Georgian_xan
    pub const KEY_Georgian_xan: Keysym = Keysym(0x010010ee);
    /// Georgian_jhan
    pub const KEY_Georgian_jhan: Keysym = Keysym(0x010010ef);
    /// Georgian_hae
    pub const KEY_Georgian_hae: Keysym = Keysym(0x010010f0);
    /// Georgian_he
    pub const KEY_Georgian_he: Keysym = Keysym(0x010010f1);
    /// Georgian_hie
    pub const KEY_Georgian_hie: Keysym = Keysym(0x010010f2);
    /// Georgian_we
    pub const KEY_Georgian_we: Keysym = Keysym(0x010010f3);
    /// Georgian_har
    pub const KEY_Georgian_har: Keysym = Keysym(0x010010f4);
    /// Georgian_hoe
    pub const KEY_Georgian_hoe: Keysym = Keysym(0x010010f5);
    /// Georgian_fi
    pub const KEY_Georgian_fi: Keysym = Keysym(0x010010f6);
    /// Xabovedot
    pub const KEY_Xabovedot: Keysym = Keysym(0x01001e8a);
    /// Ibreve
    pub const KEY_Ibreve: Keysym = Keysym(0x0100012c);
    /// Zstroke
    pub const KEY_Zstroke: Keysym = Keysym(0x010001b5);
    /// Gcaron
    pub const KEY_Gcaron: Keysym = Keysym(0x010001e6);
    /// Ocaron
    pub const KEY_Ocaron: Keysym = Keysym(0x010001d1);
    /// Obarred
    pub const KEY_Obarred: Keysym = Keysym(0x0100019f);
    /// xabovedot
    pub const KEY_xabovedot: Keysym = Keysym(0x01001e8b);
    /// ibreve
    pub const KEY_ibreve: Keysym = Keysym(0x0100012d);
    /// zstroke
    pub const KEY_zstroke: Keysym = Keysym(0x010001b6);
    /// gcaron
    pub const KEY_gcaron: Keysym = Keysym(0x010001e7);
    /// ocaron
    pub const KEY_ocaron: Keysym = Keysym(0x010001d2);
    /// obarred
    pub const KEY_obarred: Keysym = Keysym(0x01000275);
    /// SCHWA
    pub const KEY_SCHWA: Keysym = Keysym(0x0100018f);
    /// schwa
    pub const KEY_schwa: Keysym = Keysym(0x01000259);
    /// EZH
    pub const KEY_EZH: Keysym = Keysym(0x010001b7);
    /// ezh
    pub const KEY_ezh: Keysym = Keysym(0x01000292);
    /// Lbelowdot
    pub const KEY_Lbelowdot: Keysym = Keysym(0x01001e36);
    /// lbelowdot
    pub const KEY_lbelowdot: Keysym = Keysym(0x01001e37);
    /// Abelowdot
    pub const KEY_Abelowdot: Keysym = Keysym(0x01001ea0);
    /// abelowdot
    pub const KEY_abelowdot: Keysym = Keysym(0x01001ea1);
    /// Ahook
    pub const KEY_Ahook: Keysym = Keysym(0x01001ea2);
    /// ahook
    pub const KEY_ahook: Keysym = Keysym(0x01001ea3);
    /// Acircumflexacute
    pub const KEY_Acircumflexacute: Keysym = Keysym(0x01001ea4);
    /// acircumflexacute
    pub const KEY_acircumflexacute: Keysym = Keysym(0x01001ea5);
    /// Acircumflexgrave
    pub const KEY_Acircumflexgrave: Keysym = Keysym(0x01001ea6);
    /// acircumflexgrave
    pub const KEY_acircumflexgrave: Keysym = Keysym(0x01001ea7);
    /// Acircumflexhook
    pub const KEY_Acircumflexhook: Keysym = Keysym(0x01001ea8);
    /// acircumflexhook
    pub const KEY_acircumflexhook: Keysym = Keysym(0x01001ea9);
    /// Acircumflextilde
    pub const KEY_Acircumflextilde: Keysym = Keysym(0x01001eaa);
    /// acircumflextilde
    pub const KEY_acircumflextilde: Keysym = Keysym(0x01001eab);
    /// Acircumflexbelowdot
    pub const KEY_Acircumflexbelowdot: Keysym = Keysym(0x01001eac);
    /// acircumflexbelowdot
    pub const KEY_acircumflexbelowdot: Keysym = Keysym(0x01001ead);
    /// Abreveacute
    pub const KEY_Abreveacute: Keysym = Keysym(0x01001eae);
    /// abreveacute
    pub const KEY_abreveacute: Keysym = Keysym(0x01001eaf);
    /// Abrevegrave
    pub const KEY_Abrevegrave: Keysym = Keysym(0x01001eb0);
    /// abrevegrave
    pub const KEY_abrevegrave: Keysym = Keysym(0x01001eb1);
    /// Abrevehook
    pub const KEY_Abrevehook: Keysym = Keysym(0x01001eb2);
    /// abrevehook
    pub const KEY_abrevehook: Keysym = Keysym(0x01001eb3);
    /// Abrevetilde
    pub const KEY_Abrevetilde: Keysym = Keysym(0x01001eb4);
    /// abrevetilde
    pub const KEY_abrevetilde: Keysym = Keysym(0x01001eb5);
    /// Abrevebelowdot
    pub const KEY_Abrevebelowdot: Keysym = Keysym(0x01001eb6);
    /// abrevebelowdot
    pub const KEY_abrevebelowdot: Keysym = Keysym(0x01001eb7);
    /// Ebelowdot
    pub const KEY_Ebelowdot: Keysym = Keysym(0x01001eb8);
    /// ebelowdot
    pub const KEY_ebelowdot: Keysym = Keysym(0x01001eb9);
    /// Ehook
    pub const KEY_Ehook: Keysym = Keysym(0x01001eba);
    /// ehook
    pub const KEY_ehook: Keysym = Keysym(0x01001ebb);
    /// Etilde
    pub const KEY_Etilde: Keysym = Keysym(0x01001ebc);
    /// etilde
    pub const KEY_etilde: Keysym = Keysym(0x01001ebd);
    /// Ecircumflexacute
    pub const KEY_Ecircumflexacute: Keysym = Keysym(0x01001ebe);
    /// ecircumflexacute
    pub const KEY_ecircumflexacute: Keysym = Keysym(0x01001ebf);
    /// Ecircumflexgrave
    pub const KEY_Ecircumflexgrave: Keysym = Keysym(0x01001ec0);
    /// ecircumflexgrave
    pub const KEY_ecircumflexgrave: Keysym = Keysym(0x01001ec1);
    /// Ecircumflexhook
    pub const KEY_Ecircumflexhook: Keysym = Keysym(0x01001ec2);
    /// ecircumflexhook
    pub const KEY_ecircumflexhook: Keysym = Keysym(0x01001ec3);
    /// Ecircumflextilde
    pub const KEY_Ecircumflextilde: Keysym = Keysym(0x01001ec4);
    /// ecircumflextilde
    pub const KEY_ecircumflextilde: Keysym = Keysym(0x01001ec5);
    /// Ecircumflexbelowdot
    pub const KEY_Ecircumflexbelowdot: Keysym = Keysym(0x01001ec6);
    /// ecircumflexbelowdot
    pub const KEY_ecircumflexbelowdot: Keysym = Keysym(0x01001ec7);
    /// Ihook
    pub const KEY_Ihook: Keysym = Keysym(0x01001ec8);
    /// ihook
    pub const KEY_ihook: Keysym = Keysym(0x01001ec9);
    /// Ibelowdot
    pub const KEY_Ibelowdot: Keysym = Keysym(0x01001eca);
    /// ibelowdot
    pub const KEY_ibelowdot: Keysym = Keysym(0x01001ecb);
    /// Obelowdot
    pub const KEY_Obelowdot: Keysym = Keysym(0x01001ecc);
    /// obelowdot
    pub const KEY_obelowdot: Keysym = Keysym(0x01001ecd);
    /// Ohook
    pub const KEY_Ohook: Keysym = Keysym(0x01001ece);
    /// ohook
    pub const KEY_ohook: Keysym = Keysym(0x01001ecf);
    /// Ocircumflexacute
    pub const KEY_Ocircumflexacute: Keysym = Keysym(0x01001ed0);
    /// ocircumflexacute
    pub const KEY_ocircumflexacute: Keysym = Keysym(0x01001ed1);
    /// Ocircumflexgrave
    pub const KEY_Ocircumflexgrave: Keysym = Keysym(0x01001ed2);
    /// ocircumflexgrave
    pub const KEY_ocircumflexgrave: Keysym = Keysym(0x01001ed3);
    /// Ocircumflexhook
    pub const KEY_Ocircumflexhook: Keysym = Keysym(0x01001ed4);
    /// ocircumflexhook
    pub const KEY_ocircumflexhook: Keysym = Keysym(0x01001ed5);
    /// Ocircumflextilde
    pub const KEY_Ocircumflextilde: Keysym = Keysym(0x01001ed6);
    /// ocircumflextilde
    pub const KEY_ocircumflextilde: Keysym = Keysym(0x01001ed7);
    /// Ocircumflexbelowdot
    pub const KEY_Ocircumflexbelowdot: Keysym = Keysym(0x01001ed8);
    /// ocircumflexbelowdot
    pub const KEY_ocircumflexbelowdot: Keysym = Keysym(0x01001ed9);
    /// Ohornacute
    pub const KEY_Ohornacute: Keysym = Keysym(0x01001eda);
    /// ohornacute
    pub const KEY_ohornacute: Keysym = Keysym(0x01001edb);
    /// Ohorngrave
    pub const KEY_Ohorngrave: Keysym = Keysym(0x01001edc);
    /// ohorngrave
    pub const KEY_ohorngrave: Keysym = Keysym(0x01001edd);
    /// Ohornhook
    pub const KEY_Ohornhook: Keysym = Keysym(0x01001ede);
    /// ohornhook
    pub const KEY_ohornhook: Keysym = Keysym(0x01001edf);
    /// Ohorntilde
    pub const KEY_Ohorntilde: Keysym = Keysym(0x01001ee0);
    /// ohorntilde
    pub const KEY_ohorntilde: Keysym = Keysym(0x01001ee1);
    /// Ohornbelowdot
    pub const KEY_Ohornbelowdot: Keysym = Keysym(0x01001ee2);
    /// ohornbelowdot
    pub const KEY_ohornbelowdot: Keysym = Keysym(0x01001ee3);
    /// Ubelowdot
    pub const KEY_Ubelowdot: Keysym = Keysym(0x01001ee4);
    /// ubelowdot
    pub const KEY_ubelowdot: Keysym = Keysym(0x01001ee5);
    /// Uhook
    pub const KEY_Uhook: Keysym = Keysym(0x01001ee6);
    /// uhook
    pub const KEY_uhook: Keysym = Keysym(0x01001ee7);
    /// Uhornacute
    pub const KEY_Uhornacute: Keysym = Keysym(0x01001ee8);
    /// uhornacute
    pub const KEY_uhornacute: Keysym = Keysym(0x01001ee9);
    /// Uhorngrave
    pub const KEY_Uhorngrave: Keysym = Keysym(0x01001eea);
    /// uhorngrave
    pub const KEY_uhorngrave: Keysym = Keysym(0x01001eeb);
    /// Uhornhook
    pub const KEY_Uhornhook: Keysym = Keysym(0x01001eec);
    /// uhornhook
    pub const KEY_uhornhook: Keysym = Keysym(0x01001eed);
    /// Uhorntilde
    pub const KEY_Uhorntilde: Keysym = Keysym(0x01001eee);
    /// uhorntilde
    pub const KEY_uhorntilde: Keysym = Keysym(0x01001eef);
    /// Uhornbelowdot
    pub const KEY_Uhornbelowdot: Keysym = Keysym(0x01001ef0);
    /// uhornbelowdot
    pub const KEY_uhornbelowdot: Keysym = Keysym(0x01001ef1);
    /// Ybelowdot
    pub const KEY_Ybelowdot: Keysym = Keysym(0x01001ef4);
    /// ybelowdot
    pub const KEY_ybelowdot: Keysym = Keysym(0x01001ef5);
    /// Yhook
    pub const KEY_Yhook: Keysym = Keysym(0x01001ef6);
    /// yhook
    pub const KEY_yhook: Keysym = Keysym(0x01001ef7);
    /// Ytilde
    pub const KEY_Ytilde: Keysym = Keysym(0x01001ef8);
    /// ytilde
    pub const KEY_ytilde: Keysym = Keysym(0x01001ef9);
    /// Ohorn
    pub const KEY_Ohorn: Keysym = Keysym(0x010001a0);
    /// ohorn
    pub const KEY_ohorn: Keysym = Keysym(0x010001a1);
    /// Uhorn
    pub const KEY_Uhorn: Keysym = Keysym(0x010001af);
    /// uhorn
    pub const KEY_uhorn: Keysym = Keysym(0x010001b0);
    /// combining_tilde
    pub const KEY_combining_tilde: Keysym = Keysym(0x01000303);
    /// combining_grave
    pub const KEY_combining_grave: Keysym = Keysym(0x01000300);
    /// combining_acute
    pub const KEY_combining_acute: Keysym = Keysym(0x01000301);
    /// combining_hook
    pub const KEY_combining_hook: Keysym = Keysym(0x01000309);
    /// combining_belowdot
    pub const KEY_combining_belowdot: Keysym = Keysym(0x01000323);
    /// EcuSign
    pub const KEY_EcuSign: Keysym = Keysym(0x010020a0);
    /// ColonSign
    pub const KEY_ColonSign: Keysym = Keysym(0x010020a1);
    /// CruzeiroSign
    pub const KEY_CruzeiroSign: Keysym = Keysym(0x010020a2);
    /// FFrancSign
    pub const KEY_FFrancSign: Keysym = Keysym(0x010020a3);
    /// LiraSign
    pub const KEY_LiraSign: Keysym = Keysym(0x010020a4);
    /// MillSign
    pub const KEY_MillSign: Keysym = Keysym(0x010020a5);
    /// NairaSign
    pub const KEY_NairaSign: Keysym = Keysym(0x010020a6);
    /// PesetaSign
    pub const KEY_PesetaSign: Keysym = Keysym(0x010020a7);
    /// RupeeSign
    pub const KEY_RupeeSign: Keysym = Keysym(0x010020a8);
    /// WonSign
    pub const KEY_WonSign: Keysym = Keysym(0x010020a9);
    /// NewSheqelSign
    pub const KEY_NewSheqelSign: Keysym = Keysym(0x010020aa);
    /// DongSign
    pub const KEY_DongSign: Keysym = Keysym(0x010020ab);
    /// EuroSign
    pub const KEY_EuroSign: Keysym = Keysym(0x000020ac);
    /// zerosuperior
    pub const KEY_zerosuperior: Keysym = Keysym(0x01002070);
    /// foursuperior
    pub const KEY_foursuperior: Keysym = Keysym(0x01002074);
    /// fivesuperior
    pub const KEY_fivesuperior: Keysym = Keysym(0x01002075);
    /// sixsuperior
    pub const KEY_sixsuperior: Keysym = Keysym(0x01002076);
    /// sevensuperior
    pub const KEY_sevensuperior: Keysym = Keysym(0x01002077);
    /// eightsuperior
    pub const KEY_eightsuperior: Keysym = Keysym(0x01002078);
    /// ninesuperior
    pub const KEY_ninesuperior: Keysym = Keysym(0x01002079);
    /// zerosubscript
    pub const KEY_zerosubscript: Keysym = Keysym(0x01002080);
    /// onesubscript
    pub const KEY_onesubscript: Keysym = Keysym(0x01002081);
    /// twosubscript
    pub const KEY_twosubscript: Keysym = Keysym(0x01002082);
    /// threesubscript
    pub const KEY_threesubscript: Keysym = Keysym(0x01002083);
    /// foursubscript
    pub const KEY_foursubscript: Keysym = Keysym(0x01002084);
    /// fivesubscript
    pub const KEY_fivesubscript: Keysym = Keysym(0x01002085);
    /// sixsubscript
    pub const KEY_sixsubscript: Keysym = Keysym(0x01002086);
    /// sevensubscript
    pub const KEY_sevensubscript: Keysym = Keysym(0x01002087);
    /// eightsubscript
    pub const KEY_eightsubscript: Keysym = Keysym(0x01002088);
    /// ninesubscript
    pub const KEY_ninesubscript: Keysym = Keysym(0x01002089);
    /// partdifferential
    pub const KEY_partdifferential: Keysym = Keysym(0x01002202);
    /// emptyset
    pub const KEY_emptyset: Keysym = Keysym(0x01002205);
    /// elementof
    pub const KEY_elementof: Keysym = Keysym(0x01002208);
    /// notelementof
    pub const KEY_notelementof: Keysym = Keysym(0x01002209);
    /// containsas
    pub const KEY_containsas: Keysym = Keysym(0x0100220b);
    /// squareroot
    pub const KEY_squareroot: Keysym = Keysym(0x0100221a);
    /// cuberoot
    pub const KEY_cuberoot: Keysym = Keysym(0x0100221b);
    /// fourthroot
    pub const KEY_fourthroot: Keysym = Keysym(0x0100221c);
    /// dintegral
    pub const KEY_dintegral: Keysym = Keysym(0x0100222c);
    /// tintegral
    pub const KEY_tintegral: Keysym = Keysym(0x0100222d);
    /// because
    pub const KEY_because: Keysym = Keysym(0x01002235);
    /// approxeq
    pub const KEY_approxeq: Keysym = Keysym(0x01002248);
    /// notapproxeq
    pub const KEY_notapproxeq: Keysym = Keysym(0x01002247);
    /// notidentical
    pub const KEY_notidentical: Keysym = Keysym(0x01002262);
    /// stricteq
    pub const KEY_stricteq: Keysym = Keysym(0x01002263);
    /// braille_dot_1
    pub const KEY_braille_dot_1: Keysym = Keysym(0x0000fff1);
    /// braille_dot_2
    pub const KEY_braille_dot_2: Keysym = Keysym(0x0000fff2);
    /// braille_dot_3
    pub const KEY_braille_dot_3: Keysym = Keysym(0x0000fff3);
    /// braille_dot_4
    pub const KEY_braille_dot_4: Keysym = Keysym(0x0000fff4);
    /// braille_dot_5
    pub const KEY_braille_dot_5: Keysym = Keysym(0x0000fff5);
    /// braille_dot_6
    pub const KEY_braille_dot_6: Keysym = Keysym(0x0000fff6);
    /// braille_dot_7
    pub const KEY_braille_dot_7: Keysym = Keysym(0x0000fff7);
    /// braille_dot_8
    pub const KEY_braille_dot_8: Keysym = Keysym(0x0000fff8);
    /// braille_dot_9
    pub const KEY_braille_dot_9: Keysym = Keysym(0x0000fff9);
    /// braille_dot_10
    pub const KEY_braille_dot_10: Keysym = Keysym(0x0000fffa);
    /// braille_blank
    pub const KEY_braille_blank: Keysym = Keysym(0x01002800);
    /// braille_dots_1
    pub const KEY_braille_dots_1: Keysym = Keysym(0x01002801);
    /// braille_dots_2
    pub const KEY_braille_dots_2: Keysym = Keysym(0x01002802);
    /// braille_dots_12
    pub const KEY_braille_dots_12: Keysym = Keysym(0x01002803);
    /// braille_dots_3
    pub const KEY_braille_dots_3: Keysym = Keysym(0x01002804);
    /// braille_dots_13
    pub const KEY_braille_dots_13: Keysym = Keysym(0x01002805);
    /// braille_dots_23
    pub const KEY_braille_dots_23: Keysym = Keysym(0x01002806);
    /// braille_dots_123
    pub const KEY_braille_dots_123: Keysym = Keysym(0x01002807);
    /// braille_dots_4
    pub const KEY_braille_dots_4: Keysym = Keysym(0x01002808);
    /// braille_dots_14
    pub const KEY_braille_dots_14: Keysym = Keysym(0x01002809);
    /// braille_dots_24
    pub const KEY_braille_dots_24: Keysym = Keysym(0x0100280a);
    /// braille_dots_124
    pub const KEY_braille_dots_124: Keysym = Keysym(0x0100280b);
    /// braille_dots_34
    pub const KEY_braille_dots_34: Keysym = Keysym(0x0100280c);
    /// braille_dots_134
    pub const KEY_braille_dots_134: Keysym = Keysym(0x0100280d);
    /// braille_dots_234
    pub const KEY_braille_dots_234: Keysym = Keysym(0x0100280e);
    /// braille_dots_1234
    pub const KEY_braille_dots_1234: Keysym = Keysym(0x0100280f);
    /// braille_dots_5
    pub const KEY_braille_dots_5: Keysym = Keysym(0x01002810);
    /// braille_dots_15
    pub const KEY_braille_dots_15: Keysym = Keysym(0x01002811);
    /// braille_dots_25
    pub const KEY_braille_dots_25: Keysym = Keysym(0x01002812);
    /// braille_dots_125
    pub const KEY_braille_dots_125: Keysym = Keysym(0x01002813);
    /// braille_dots_35
    pub const KEY_braille_dots_35: Keysym = Keysym(0x01002814);
    /// braille_dots_135
    pub const KEY_braille_dots_135: Keysym = Keysym(0x01002815);
    /// braille_dots_235
    pub const KEY_braille_dots_235: Keysym = Keysym(0x01002816);
    /// braille_dots_1235
    pub const KEY_braille_dots_1235: Keysym = Keysym(0x01002817);
    /// braille_dots_45
    pub const KEY_braille_dots_45: Keysym = Keysym(0x01002818);
    /// braille_dots_145
    pub const KEY_braille_dots_145: Keysym = Keysym(0x01002819);
    /// braille_dots_245
    pub const KEY_braille_dots_245: Keysym = Keysym(0x0100281a);
    /// braille_dots_1245
    pub const KEY_braille_dots_1245: Keysym = Keysym(0x0100281b);
    /// braille_dots_345
    pub const KEY_braille_dots_345: Keysym = Keysym(0x0100281c);
    /// braille_dots_1345
    pub const KEY_braille_dots_1345: Keysym = Keysym(0x0100281d);
    /// braille_dots_2345
    pub const KEY_braille_dots_2345: Keysym = Keysym(0x0100281e);
    /// braille_dots_12345
    pub const KEY_braille_dots_12345: Keysym = Keysym(0x0100281f);
    /// braille_dots_6
    pub const KEY_braille_dots_6: Keysym = Keysym(0x01002820);
    /// braille_dots_16
    pub const KEY_braille_dots_16: Keysym = Keysym(0x01002821);
    /// braille_dots_26
    pub const KEY_braille_dots_26: Keysym = Keysym(0x01002822);
    /// braille_dots_126
    pub const KEY_braille_dots_126: Keysym = Keysym(0x01002823);
    /// braille_dots_36
    pub const KEY_braille_dots_36: Keysym = Keysym(0x01002824);
    /// braille_dots_136
    pub const KEY_braille_dots_136: Keysym = Keysym(0x01002825);
    /// braille_dots_236
    pub const KEY_braille_dots_236: Keysym = Keysym(0x01002826);
    /// braille_dots_1236
    pub const KEY_braille_dots_1236: Keysym = Keysym(0x01002827);
    /// braille_dots_46
    pub const KEY_braille_dots_46: Keysym = Keysym(0x01002828);
    /// braille_dots_146
    pub const KEY_braille_dots_146: Keysym = Keysym(0x01002829);
    /// braille_dots_246
    pub const KEY_braille_dots_246: Keysym = Keysym(0x0100282a);
    /// braille_dots_1246
    pub const KEY_braille_dots_1246: Keysym = Keysym(0x0100282b);
    /// braille_dots_346
    pub const KEY_braille_dots_346: Keysym = Keysym(0x0100282c);
    /// braille_dots_1346
    pub const KEY_braille_dots_1346: Keysym = Keysym(0x0100282d);
    /// braille_dots_2346
    pub const KEY_braille_dots_2346: Keysym = Keysym(0x0100282e);
    /// braille_dots_12346
    pub const KEY_braille_dots_12346: Keysym = Keysym(0x0100282f);
    /// braille_dots_56
    pub const KEY_braille_dots_56: Keysym = Keysym(0x01002830);
    /// braille_dots_156
    pub const KEY_braille_dots_156: Keysym = Keysym(0x01002831);
    /// braille_dots_256
    pub const KEY_braille_dots_256: Keysym = Keysym(0x01002832);
    /// braille_dots_1256
    pub const KEY_braille_dots_1256: Keysym = Keysym(0x01002833);
    /// braille_dots_356
    pub const KEY_braille_dots_356: Keysym = Keysym(0x01002834);
    /// braille_dots_1356
    pub const KEY_braille_dots_1356: Keysym = Keysym(0x01002835);
    /// braille_dots_2356
    pub const KEY_braille_dots_2356: Keysym = Keysym(0x01002836);
    /// braille_dots_12356
    pub const KEY_braille_dots_12356: Keysym = Keysym(0x01002837);
    /// braille_dots_456
    pub const KEY_braille_dots_456: Keysym = Keysym(0x01002838);
    /// braille_dots_1456
    pub const KEY_braille_dots_1456: Keysym = Keysym(0x01002839);
    /// braille_dots_2456
    pub const KEY_braille_dots_2456: Keysym = Keysym(0x0100283a);
    /// braille_dots_12456
    pub const KEY_braille_dots_12456: Keysym = Keysym(0x0100283b);
    /// braille_dots_3456
    pub const KEY_braille_dots_3456: Keysym = Keysym(0x0100283c);
    /// braille_dots_13456
    pub const KEY_braille_dots_13456: Keysym = Keysym(0x0100283d);
    /// braille_dots_23456
    pub const KEY_braille_dots_23456: Keysym = Keysym(0x0100283e);
    /// braille_dots_123456
    pub const KEY_braille_dots_123456: Keysym = Keysym(0x0100283f);
    /// braille_dots_7
    pub const KEY_braille_dots_7: Keysym = Keysym(0x01002840);
    /// braille_dots_17
    pub const KEY_braille_dots_17: Keysym = Keysym(0x01002841);
    /// braille_dots_27
    pub const KEY_braille_dots_27: Keysym = Keysym(0x01002842);
    /// braille_dots_127
    pub const KEY_braille_dots_127: Keysym = Keysym(0x01002843);
    /// braille_dots_37
    pub const KEY_braille_dots_37: Keysym = Keysym(0x01002844);
    /// braille_dots_137
    pub const KEY_braille_dots_137: Keysym = Keysym(0x01002845);
    /// braille_dots_237
    pub const KEY_braille_dots_237: Keysym = Keysym(0x01002846);
    /// braille_dots_1237
    pub const KEY_braille_dots_1237: Keysym = Keysym(0x01002847);
    /// braille_dots_47
    pub const KEY_braille_dots_47: Keysym = Keysym(0x01002848);
    /// braille_dots_147
    pub const KEY_braille_dots_147: Keysym = Keysym(0x01002849);
    /// braille_dots_247
    pub const KEY_braille_dots_247: Keysym = Keysym(0x0100284a);
    /// braille_dots_1247
    pub const KEY_braille_dots_1247: Keysym = Keysym(0x0100284b);
    /// braille_dots_347
    pub const KEY_braille_dots_347: Keysym = Keysym(0x0100284c);
    /// braille_dots_1347
    pub const KEY_braille_dots_1347: Keysym = Keysym(0x0100284d);
    /// braille_dots_2347
    pub const KEY_braille_dots_2347: Keysym = Keysym(0x0100284e);
    /// braille_dots_12347
    pub const KEY_braille_dots_12347: Keysym = Keysym(0x0100284f);
    /// braille_dots_57
    pub const KEY_braille_dots_57: Keysym = Keysym(0x01002850);
    /// braille_dots_157
    pub const KEY_braille_dots_157: Keysym = Keysym(0x01002851);
    /// braille_dots_257
    pub const KEY_braille_dots_257: Keysym = Keysym(0x01002852);
    /// braille_dots_1257
    pub const KEY_braille_dots_1257: Keysym = Keysym(0x01002853);
    /// braille_dots_357
    pub const KEY_braille_dots_357: Keysym = Keysym(0x01002854);
    /// braille_dots_1357
    pub const KEY_braille_dots_1357: Keysym = Keysym(0x01002855);
    /// braille_dots_2357
    pub const KEY_braille_dots_2357: Keysym = Keysym(0x01002856);
    /// braille_dots_12357
    pub const KEY_braille_dots_12357: Keysym = Keysym(0x01002857);
    /// braille_dots_457
    pub const KEY_braille_dots_457: Keysym = Keysym(0x01002858);
    /// braille_dots_1457
    pub const KEY_braille_dots_1457: Keysym = Keysym(0x01002859);
    /// braille_dots_2457
    pub const KEY_braille_dots_2457: Keysym = Keysym(0x0100285a);
    /// braille_dots_12457
    pub const KEY_braille_dots_12457: Keysym = Keysym(0x0100285b);
    /// braille_dots_3457
    pub const KEY_braille_dots_3457: Keysym = Keysym(0x0100285c);
    /// braille_dots_13457
    pub const KEY_braille_dots_13457: Keysym = Keysym(0x0100285d);
    /// braille_dots_23457
    pub const KEY_braille_dots_23457: Keysym = Keysym(0x0100285e);
    /// braille_dots_123457
    pub const KEY_braille_dots_123457: Keysym = Keysym(0x0100285f);
    /// braille_dots_67
    pub const KEY_braille_dots_67: Keysym = Keysym(0x01002860);
    /// braille_dots_167
    pub const KEY_braille_dots_167: Keysym = Keysym(0x01002861);
    /// braille_dots_267
    pub const KEY_braille_dots_267: Keysym = Keysym(0x01002862);
    /// braille_dots_1267
    pub const KEY_braille_dots_1267: Keysym = Keysym(0x01002863);
    /// braille_dots_367
    pub const KEY_braille_dots_367: Keysym = Keysym(0x01002864);
    /// braille_dots_1367
    pub const KEY_braille_dots_1367: Keysym = Keysym(0x01002865);
    /// braille_dots_2367
    pub const KEY_braille_dots_2367: Keysym = Keysym(0x01002866);
    /// braille_dots_12367
    pub const KEY_braille_dots_12367: Keysym = Keysym(0x01002867);
    /// braille_dots_467
    pub const KEY_braille_dots_467: Keysym = Keysym(0x01002868);
    /// braille_dots_1467
    pub const KEY_braille_dots_1467: Keysym = Keysym(0x01002869);
    /// braille_dots_2467
    pub const KEY_braille_dots_2467: Keysym = Keysym(0x0100286a);
    /// braille_dots_12467
    pub const KEY_braille_dots_12467: Keysym = Keysym(0x0100286b);
    /// braille_dots_3467
    pub const KEY_braille_dots_3467: Keysym = Keysym(0x0100286c);
    /// braille_dots_13467
    pub const KEY_braille_dots_13467: Keysym = Keysym(0x0100286d);
    /// braille_dots_23467
    pub const KEY_braille_dots_23467: Keysym = Keysym(0x0100286e);
    /// braille_dots_123467
    pub const KEY_braille_dots_123467: Keysym = Keysym(0x0100286f);
    /// braille_dots_567
    pub const KEY_braille_dots_567: Keysym = Keysym(0x01002870);
    /// braille_dots_1567
    pub const KEY_braille_dots_1567: Keysym = Keysym(0x01002871);
    /// braille_dots_2567
    pub const KEY_braille_dots_2567: Keysym = Keysym(0x01002872);
    /// braille_dots_12567
    pub const KEY_braille_dots_12567: Keysym = Keysym(0x01002873);
    /// braille_dots_3567
    pub const KEY_braille_dots_3567: Keysym = Keysym(0x01002874);
    /// braille_dots_13567
    pub const KEY_braille_dots_13567: Keysym = Keysym(0x01002875);
    /// braille_dots_23567
    pub const KEY_braille_dots_23567: Keysym = Keysym(0x01002876);
    /// braille_dots_123567
    pub const KEY_braille_dots_123567: Keysym = Keysym(0x01002877);
    /// braille_dots_4567
    pub const KEY_braille_dots_4567: Keysym = Keysym(0x01002878);
    /// braille_dots_14567
    pub const KEY_braille_dots_14567: Keysym = Keysym(0x01002879);
    /// braille_dots_24567
    pub const KEY_braille_dots_24567: Keysym = Keysym(0x0100287a);
    /// braille_dots_124567
    pub const KEY_braille_dots_124567: Keysym = Keysym(0x0100287b);
    /// braille_dots_34567
    pub const KEY_braille_dots_34567: Keysym = Keysym(0x0100287c);
    /// braille_dots_134567
    pub const KEY_braille_dots_134567: Keysym = Keysym(0x0100287d);
    /// braille_dots_234567
    pub const KEY_braille_dots_234567: Keysym = Keysym(0x0100287e);
    /// braille_dots_1234567
    pub const KEY_braille_dots_1234567: Keysym = Keysym(0x0100287f);
    /// braille_dots_8
    pub const KEY_braille_dots_8: Keysym = Keysym(0x01002880);
    /// braille_dots_18
    pub const KEY_braille_dots_18: Keysym = Keysym(0x01002881);
    /// braille_dots_28
    pub const KEY_braille_dots_28: Keysym = Keysym(0x01002882);
    /// braille_dots_128
    pub const KEY_braille_dots_128: Keysym = Keysym(0x01002883);
    /// braille_dots_38
    pub const KEY_braille_dots_38: Keysym = Keysym(0x01002884);
    /// braille_dots_138
    pub const KEY_braille_dots_138: Keysym = Keysym(0x01002885);
    /// braille_dots_238
    pub const KEY_braille_dots_238: Keysym = Keysym(0x01002886);
    /// braille_dots_1238
    pub const KEY_braille_dots_1238: Keysym = Keysym(0x01002887);
    /// braille_dots_48
    pub const KEY_braille_dots_48: Keysym = Keysym(0x01002888);
    /// braille_dots_148
    pub const KEY_braille_dots_148: Keysym = Keysym(0x01002889);
    /// braille_dots_248
    pub const KEY_braille_dots_248: Keysym = Keysym(0x0100288a);
    /// braille_dots_1248
    pub const KEY_braille_dots_1248: Keysym = Keysym(0x0100288b);
    /// braille_dots_348
    pub const KEY_braille_dots_348: Keysym = Keysym(0x0100288c);
    /// braille_dots_1348
    pub const KEY_braille_dots_1348: Keysym = Keysym(0x0100288d);
    /// braille_dots_2348
    pub const KEY_braille_dots_2348: Keysym = Keysym(0x0100288e);
    /// braille_dots_12348
    pub const KEY_braille_dots_12348: Keysym = Keysym(0x0100288f);
    /// braille_dots_58
    pub const KEY_braille_dots_58: Keysym = Keysym(0x01002890);
    /// braille_dots_158
    pub const KEY_braille_dots_158: Keysym = Keysym(0x01002891);
    /// braille_dots_258
    pub const KEY_braille_dots_258: Keysym = Keysym(0x01002892);
    /// braille_dots_1258
    pub const KEY_braille_dots_1258: Keysym = Keysym(0x01002893);
    /// braille_dots_358
    pub const KEY_braille_dots_358: Keysym = Keysym(0x01002894);
    /// braille_dots_1358
    pub const KEY_braille_dots_1358: Keysym = Keysym(0x01002895);
    /// braille_dots_2358
    pub const KEY_braille_dots_2358: Keysym = Keysym(0x01002896);
    /// braille_dots_12358
    pub const KEY_braille_dots_12358: Keysym = Keysym(0x01002897);
    /// braille_dots_458
    pub const KEY_braille_dots_458: Keysym = Keysym(0x01002898);
    /// braille_dots_1458
    pub const KEY_braille_dots_1458: Keysym = Keysym(0x01002899);
    /// braille_dots_2458
    pub const KEY_braille_dots_2458: Keysym = Keysym(0x0100289a);
    /// braille_dots_12458
    pub const KEY_braille_dots_12458: Keysym = Keysym(0x0100289b);
    /// braille_dots_3458
    pub const KEY_braille_dots_3458: Keysym = Keysym(0x0100289c);
    /// braille_dots_13458
    pub const KEY_braille_dots_13458: Keysym = Keysym(0x0100289d);
    /// braille_dots_23458
    pub const KEY_braille_dots_23458: Keysym = Keysym(0x0100289e);
    /// braille_dots_123458
    pub const KEY_braille_dots_123458: Keysym = Keysym(0x0100289f);
    /// braille_dots_68
    pub const KEY_braille_dots_68: Keysym = Keysym(0x010028a0);
    /// braille_dots_168
    pub const KEY_braille_dots_168: Keysym = Keysym(0x010028a1);
    /// braille_dots_268
    pub const KEY_braille_dots_268: Keysym = Keysym(0x010028a2);
    /// braille_dots_1268
    pub const KEY_braille_dots_1268: Keysym = Keysym(0x010028a3);
    /// braille_dots_368
    pub const KEY_braille_dots_368: Keysym = Keysym(0x010028a4);
    /// braille_dots_1368
    pub const KEY_braille_dots_1368: Keysym = Keysym(0x010028a5);
    /// braille_dots_2368
    pub const KEY_braille_dots_2368: Keysym = Keysym(0x010028a6);
    /// braille_dots_12368
    pub const KEY_braille_dots_12368: Keysym = Keysym(0x010028a7);
    /// braille_dots_468
    pub const KEY_braille_dots_468: Keysym = Keysym(0x010028a8);
    /// braille_dots_1468
    pub const KEY_braille_dots_1468: Keysym = Keysym(0x010028a9);
    /// braille_dots_2468
    pub const KEY_braille_dots_2468: Keysym = Keysym(0x010028aa);
    /// braille_dots_12468
    pub const KEY_braille_dots_12468: Keysym = Keysym(0x010028ab);
    /// braille_dots_3468
    pub const KEY_braille_dots_3468: Keysym = Keysym(0x010028ac);
    /// braille_dots_13468
    pub const KEY_braille_dots_13468: Keysym = Keysym(0x010028ad);
    /// braille_dots_23468
    pub const KEY_braille_dots_23468: Keysym = Keysym(0x010028ae);
    /// braille_dots_123468
    pub const KEY_braille_dots_123468: Keysym = Keysym(0x010028af);
    /// braille_dots_568
    pub const KEY_braille_dots_568: Keysym = Keysym(0x010028b0);
    /// braille_dots_1568
    pub const KEY_braille_dots_1568: Keysym = Keysym(0x010028b1);
    /// braille_dots_2568
    pub const KEY_braille_dots_2568: Keysym = Keysym(0x010028b2);
    /// braille_dots_12568
    pub const KEY_braille_dots_12568: Keysym = Keysym(0x010028b3);
    /// braille_dots_3568
    pub const KEY_braille_dots_3568: Keysym = Keysym(0x010028b4);
    /// braille_dots_13568
    pub const KEY_braille_dots_13568: Keysym = Keysym(0x010028b5);
    /// braille_dots_23568
    pub const KEY_braille_dots_23568: Keysym = Keysym(0x010028b6);
    /// braille_dots_123568
    pub const KEY_braille_dots_123568: Keysym = Keysym(0x010028b7);
    /// braille_dots_4568
    pub const KEY_braille_dots_4568: Keysym = Keysym(0x010028b8);
    /// braille_dots_14568
    pub const KEY_braille_dots_14568: Keysym = Keysym(0x010028b9);
    /// braille_dots_24568
    pub const KEY_braille_dots_24568: Keysym = Keysym(0x010028ba);
    /// braille_dots_124568
    pub const KEY_braille_dots_124568: Keysym = Keysym(0x010028bb);
    /// braille_dots_34568
    pub const KEY_braille_dots_34568: Keysym = Keysym(0x010028bc);
    /// braille_dots_134568
    pub const KEY_braille_dots_134568: Keysym = Keysym(0x010028bd);
    /// braille_dots_234568
    pub const KEY_braille_dots_234568: Keysym = Keysym(0x010028be);
    /// braille_dots_1234568
    pub const KEY_braille_dots_1234568: Keysym = Keysym(0x010028bf);
    /// braille_dots_78
    pub const KEY_braille_dots_78: Keysym = Keysym(0x010028c0);
    /// braille_dots_178
    pub const KEY_braille_dots_178: Keysym = Keysym(0x010028c1);
    /// braille_dots_278
    pub const KEY_braille_dots_278: Keysym = Keysym(0x010028c2);
    /// braille_dots_1278
    pub const KEY_braille_dots_1278: Keysym = Keysym(0x010028c3);
    /// braille_dots_378
    pub const KEY_braille_dots_378: Keysym = Keysym(0x010028c4);
    /// braille_dots_1378
    pub const KEY_braille_dots_1378: Keysym = Keysym(0x010028c5);
    /// braille_dots_2378
    pub const KEY_braille_dots_2378: Keysym = Keysym(0x010028c6);
    /// braille_dots_12378
    pub const KEY_braille_dots_12378: Keysym = Keysym(0x010028c7);
    /// braille_dots_478
    pub const KEY_braille_dots_478: Keysym = Keysym(0x010028c8);
    /// braille_dots_1478
    pub const KEY_braille_dots_1478: Keysym = Keysym(0x010028c9);
    /// braille_dots_2478
    pub const KEY_braille_dots_2478: Keysym = Keysym(0x010028ca);
    /// braille_dots_12478
    pub const KEY_braille_dots_12478: Keysym = Keysym(0x010028cb);
    /// braille_dots_3478
    pub const KEY_braille_dots_3478: Keysym = Keysym(0x010028cc);
    /// braille_dots_13478
    pub const KEY_braille_dots_13478: Keysym = Keysym(0x010028cd);
    /// braille_dots_23478
    pub const KEY_braille_dots_23478: Keysym = Keysym(0x010028ce);
    /// braille_dots_123478
    pub const KEY_braille_dots_123478: Keysym = Keysym(0x010028cf);
    /// braille_dots_578
    pub const KEY_braille_dots_578: Keysym = Keysym(0x010028d0);
    /// braille_dots_1578
    pub const KEY_braille_dots_1578: Keysym = Keysym(0x010028d1);
    /// braille_dots_2578
    pub const KEY_braille_dots_2578: Keysym = Keysym(0x010028d2);
    /// braille_dots_12578
    pub const KEY_braille_dots_12578: Keysym = Keysym(0x010028d3);
    /// braille_dots_3578
    pub const KEY_braille_dots_3578: Keysym = Keysym(0x010028d4);
    /// braille_dots_13578
    pub const KEY_braille_dots_13578: Keysym = Keysym(0x010028d5);
    /// braille_dots_23578
    pub const KEY_braille_dots_23578: Keysym = Keysym(0x010028d6);
    /// braille_dots_123578
    pub const KEY_braille_dots_123578: Keysym = Keysym(0x010028d7);
    /// braille_dots_4578
    pub const KEY_braille_dots_4578: Keysym = Keysym(0x010028d8);
    /// braille_dots_14578
    pub const KEY_braille_dots_14578: Keysym = Keysym(0x010028d9);
    /// braille_dots_24578
    pub const KEY_braille_dots_24578: Keysym = Keysym(0x010028da);
    /// braille_dots_124578
    pub const KEY_braille_dots_124578: Keysym = Keysym(0x010028db);
    /// braille_dots_34578
    pub const KEY_braille_dots_34578: Keysym = Keysym(0x010028dc);
    /// braille_dots_134578
    pub const KEY_braille_dots_134578: Keysym = Keysym(0x010028dd);
    /// braille_dots_234578
    pub const KEY_braille_dots_234578: Keysym = Keysym(0x010028de);
    /// braille_dots_1234578
    pub const KEY_braille_dots_1234578: Keysym = Keysym(0x010028df);
    /// braille_dots_678
    pub const KEY_braille_dots_678: Keysym = Keysym(0x010028e0);
    /// braille_dots_1678
    pub const KEY_braille_dots_1678: Keysym = Keysym(0x010028e1);
    /// braille_dots_2678
    pub const KEY_braille_dots_2678: Keysym = Keysym(0x010028e2);
    /// braille_dots_12678
    pub const KEY_braille_dots_12678: Keysym = Keysym(0x010028e3);
    /// braille_dots_3678
    pub const KEY_braille_dots_3678: Keysym = Keysym(0x010028e4);
    /// braille_dots_13678
    pub const KEY_braille_dots_13678: Keysym = Keysym(0x010028e5);
    /// braille_dots_23678
    pub const KEY_braille_dots_23678: Keysym = Keysym(0x010028e6);
    /// braille_dots_123678
    pub const KEY_braille_dots_123678: Keysym = Keysym(0x010028e7);
    /// braille_dots_4678
    pub const KEY_braille_dots_4678: Keysym = Keysym(0x010028e8);
    /// braille_dots_14678
    pub const KEY_braille_dots_14678: Keysym = Keysym(0x010028e9);
    /// braille_dots_24678
    pub const KEY_braille_dots_24678: Keysym = Keysym(0x010028ea);
    /// braille_dots_124678
    pub const KEY_braille_dots_124678: Keysym = Keysym(0x010028eb);
    /// braille_dots_34678
    pub const KEY_braille_dots_34678: Keysym = Keysym(0x010028ec);
    /// braille_dots_134678
    pub const KEY_braille_dots_134678: Keysym = Keysym(0x010028ed);
    /// braille_dots_234678
    pub const KEY_braille_dots_234678: Keysym = Keysym(0x010028ee);
    /// braille_dots_1234678
    pub const KEY_braille_dots_1234678: Keysym = Keysym(0x010028ef);
    /// braille_dots_5678
    pub const KEY_braille_dots_5678: Keysym = Keysym(0x010028f0);
    /// braille_dots_15678
    pub const KEY_braille_dots_15678: Keysym = Keysym(0x010028f1);
    /// braille_dots_25678
    pub const KEY_braille_dots_25678: Keysym = Keysym(0x010028f2);
    /// braille_dots_125678
    pub const KEY_braille_dots_125678: Keysym = Keysym(0x010028f3);
    /// braille_dots_35678
    pub const KEY_braille_dots_35678: Keysym = Keysym(0x010028f4);
    /// braille_dots_135678
    pub const KEY_braille_dots_135678: Keysym = Keysym(0x010028f5);
    /// braille_dots_235678
    pub const KEY_braille_dots_235678: Keysym = Keysym(0x010028f6);
    /// braille_dots_1235678
    pub const KEY_braille_dots_1235678: Keysym = Keysym(0x010028f7);
    /// braille_dots_45678
    pub const KEY_braille_dots_45678: Keysym = Keysym(0x010028f8);
    /// braille_dots_145678
    pub const KEY_braille_dots_145678: Keysym = Keysym(0x010028f9);
    /// braille_dots_245678
    pub const KEY_braille_dots_245678: Keysym = Keysym(0x010028fa);
    /// braille_dots_1245678
    pub const KEY_braille_dots_1245678: Keysym = Keysym(0x010028fb);
    /// braille_dots_345678
    pub const KEY_braille_dots_345678: Keysym = Keysym(0x010028fc);
    /// braille_dots_1345678
    pub const KEY_braille_dots_1345678: Keysym = Keysym(0x010028fd);
    /// braille_dots_2345678
    pub const KEY_braille_dots_2345678: Keysym = Keysym(0x010028fe);
    /// braille_dots_12345678
    pub const KEY_braille_dots_12345678: Keysym = Keysym(0x010028ff);
    /// Sinh_ng
    pub const KEY_Sinh_ng: Keysym = Keysym(0x01000d82);
    /// Sinh_h2
    pub const KEY_Sinh_h2: Keysym = Keysym(0x01000d83);
    /// Sinh_a
    pub const KEY_Sinh_a: Keysym = Keysym(0x01000d85);
    /// Sinh_aa
    pub const KEY_Sinh_aa: Keysym = Keysym(0x01000d86);
    /// Sinh_ae
    pub const KEY_Sinh_ae: Keysym = Keysym(0x01000d87);
    /// Sinh_aee
    pub const KEY_Sinh_aee: Keysym = Keysym(0x01000d88);
    /// Sinh_i
    pub const KEY_Sinh_i: Keysym = Keysym(0x01000d89);
    /// Sinh_ii
    pub const KEY_Sinh_ii: Keysym = Keysym(0x01000d8a);
    /// Sinh_u
    pub const KEY_Sinh_u: Keysym = Keysym(0x01000d8b);
    /// Sinh_uu
    pub const KEY_Sinh_uu: Keysym = Keysym(0x01000d8c);
    /// Sinh_ri
    pub const KEY_Sinh_ri: Keysym = Keysym(0x01000d8d);
    /// Sinh_rii
    pub const KEY_Sinh_rii: Keysym = Keysym(0x01000d8e);
    /// Sinh_lu
    pub const KEY_Sinh_lu: Keysym = Keysym(0x01000d8f);
    /// Sinh_luu
    pub const KEY_Sinh_luu: Keysym = Keysym(0x01000d90);
    /// Sinh_e
    pub const KEY_Sinh_e: Keysym = Keysym(0x01000d91);
    /// Sinh_ee
    pub const KEY_Sinh_ee: Keysym = Keysym(0x01000d92);
    /// Sinh_ai
    pub const KEY_Sinh_ai: Keysym = Keysym(0x01000d93);
    /// Sinh_o
    pub const KEY_Sinh_o: Keysym = Keysym(0x01000d94);
    /// Sinh_oo
    pub const KEY_Sinh_oo: Keysym = Keysym(0x01000d95);
    /// Sinh_au
    pub const KEY_Sinh_au: Keysym = Keysym(0x01000d96);
    /// Sinh_ka
    pub const KEY_Sinh_ka: Keysym = Keysym(0x01000d9a);
    /// Sinh_kha
    pub const KEY_Sinh_kha: Keysym = Keysym(0x01000d9b);
    /// Sinh_ga
    pub const KEY_Sinh_ga: Keysym = Keysym(0x01000d9c);
    /// Sinh_gha
    pub const KEY_Sinh_gha: Keysym = Keysym(0x01000d9d);
    /// Sinh_ng2
    pub const KEY_Sinh_ng2: Keysym = Keysym(0x01000d9e);
    /// Sinh_nga
    pub const KEY_Sinh_nga: Keysym = Keysym(0x01000d9f);
    /// Sinh_ca
    pub const KEY_Sinh_ca: Keysym = Keysym(0x01000da0);
    /// Sinh_cha
    pub const KEY_Sinh_cha: Keysym = Keysym(0x01000da1);
    /// Sinh_ja
    pub const KEY_Sinh_ja: Keysym = Keysym(0x01000da2);
    /// Sinh_jha
    pub const KEY_Sinh_jha: Keysym = Keysym(0x01000da3);
    /// Sinh_nya
    pub const KEY_Sinh_nya: Keysym = Keysym(0x01000da4);
    /// Sinh_jnya
    pub const KEY_Sinh_jnya: Keysym = Keysym(0x01000da5);
    /// Sinh_nja
    pub const KEY_Sinh_nja: Keysym = Keysym(0x01000da6);
    /// Sinh_tta
    pub const KEY_Sinh_tta: Keysym = Keysym(0x01000da7);
    /// Sinh_ttha
    pub const KEY_Sinh_ttha: Keysym = Keysym(0x01000da8);
    /// Sinh_dda
    pub const KEY_Sinh_dda: Keysym = Keysym(0x01000da9);
    /// Sinh_ddha
    pub const KEY_Sinh_ddha: Keysym = Keysym(0x01000daa);
    /// Sinh_nna
    pub const KEY_Sinh_nna: Keysym = Keysym(0x01000dab);
    /// Sinh_ndda
    pub const KEY_Sinh_ndda: Keysym = Keysym(0x01000dac);
    /// Sinh_tha
    pub const KEY_Sinh_tha: Keysym = Keysym(0x01000dad);
    /// Sinh_thha
    pub const KEY_Sinh_thha: Keysym = Keysym(0x01000dae);
    /// Sinh_dha
    pub const KEY_Sinh_dha: Keysym = Keysym(0x01000daf);
    /// Sinh_dhha
    pub const KEY_Sinh_dhha: Keysym = Keysym(0x01000db0);
    /// Sinh_na
    pub const KEY_Sinh_na: Keysym = Keysym(0x01000db1);
    /// Sinh_ndha
    pub const KEY_Sinh_ndha: Keysym = Keysym(0x01000db3);
    /// Sinh_pa
    pub const KEY_Sinh_pa: Keysym = Keysym(0x01000db4);
    /// Sinh_pha
    pub const KEY_Sinh_pha: Keysym = Keysym(0x01000db5);
    /// Sinh_ba
    pub const KEY_Sinh_ba: Keysym = Keysym(0x01000db6);
    /// Sinh_bha
    pub const KEY_Sinh_bha: Keysym = Keysym(0x01000db7);
    /// Sinh_ma
    pub const KEY_Sinh_ma: Keysym = Keysym(0x01000db8);
    /// Sinh_mba
    pub const KEY_Sinh_mba: Keysym = Keysym(0x01000db9);
    /// Sinh_ya
    pub const KEY_Sinh_ya: Keysym = Keysym(0x01000dba);
    /// Sinh_ra
    pub const KEY_Sinh_ra: Keysym = Keysym(0x01000dbb);
    /// Sinh_la
    pub const KEY_Sinh_la: Keysym = Keysym(0x01000dbd);
    /// Sinh_va
    pub const KEY_Sinh_va: Keysym = Keysym(0x01000dc0);
    /// Sinh_sha
    pub const KEY_Sinh_sha: Keysym = Keysym(0x01000dc1);
    /// Sinh_ssha
    pub const KEY_Sinh_ssha: Keysym = Keysym(0x01000dc2);
    /// Sinh_sa
    pub const KEY_Sinh_sa: Keysym = Keysym(0x01000dc3);
    /// Sinh_ha
    pub const KEY_Sinh_ha: Keysym = Keysym(0x01000dc4);
    /// Sinh_lla
    pub const KEY_Sinh_lla: Keysym = Keysym(0x01000dc5);
    /// Sinh_fa
    pub const KEY_Sinh_fa: Keysym = Keysym(0x01000dc6);
    /// Sinh_al
    pub const KEY_Sinh_al: Keysym = Keysym(0x01000dca);
    /// Sinh_aa2
    pub const KEY_Sinh_aa2: Keysym = Keysym(0x01000dcf);
    /// Sinh_ae2
    pub const KEY_Sinh_ae2: Keysym = Keysym(0x01000dd0);
    /// Sinh_aee2
    pub const KEY_Sinh_aee2: Keysym = Keysym(0x01000dd1);
    /// Sinh_i2
    pub const KEY_Sinh_i2: Keysym = Keysym(0x01000dd2);
    /// Sinh_ii2
    pub const KEY_Sinh_ii2: Keysym = Keysym(0x01000dd3);
    /// Sinh_u2
    pub const KEY_Sinh_u2: Keysym = Keysym(0x01000dd4);
    /// Sinh_uu2
    pub const KEY_Sinh_uu2: Keysym = Keysym(0x01000dd6);
    /// Sinh_ru2
    pub const KEY_Sinh_ru2: Keysym = Keysym(0x01000dd8);
    /// Sinh_e2
    pub const KEY_Sinh_e2: Keysym = Keysym(0x01000dd9);
    /// Sinh_ee2
    pub const KEY_Sinh_ee2: Keysym = Keysym(0x01000dda);
    /// Sinh_ai2
    pub const KEY_Sinh_ai2: Keysym = Keysym(0x01000ddb);
    /// Sinh_o2
    pub const KEY_Sinh_o2: Keysym = Keysym(0x01000ddc);
    /// Sinh_oo2
    pub const KEY_Sinh_oo2: Keysym = Keysym(0x01000ddd);
    /// Sinh_au2
    pub const KEY_Sinh_au2: Keysym = Keysym(0x01000dde);
    /// Sinh_lu2
    pub const KEY_Sinh_lu2: Keysym = Keysym(0x01000ddf);
    /// Sinh_ruu2
    pub const KEY_Sinh_ruu2: Keysym = Keysym(0x01000df2);
    /// Sinh_luu2
    pub const KEY_Sinh_luu2: Keysym = Keysym(0x01000df3);
    /// Sinh_kunddaliya
    pub const KEY_Sinh_kunddaliya: Keysym = Keysym(0x01000df4);
    /// XF86ModeLock
    pub const KEY_XF86ModeLock: Keysym = Keysym(0x1008ff01);
    /// XF86MonBrightnessUp
    pub const KEY_XF86MonBrightnessUp: Keysym = Keysym(0x1008ff02);
    /// XF86MonBrightnessDown
    pub const KEY_XF86MonBrightnessDown: Keysym = Keysym(0x1008ff03);
    /// XF86KbdLightOnOff
    pub const KEY_XF86KbdLightOnOff: Keysym = Keysym(0x1008ff04);
    /// XF86KbdBrightnessUp
    pub const KEY_XF86KbdBrightnessUp: Keysym = Keysym(0x1008ff05);
    /// XF86KbdBrightnessDown
    pub const KEY_XF86KbdBrightnessDown: Keysym = Keysym(0x1008ff06);
    /// XF86MonBrightnessCycle
    pub const KEY_XF86MonBrightnessCycle: Keysym = Keysym(0x1008ff07);
    /// XF86Standby
    pub const KEY_XF86Standby: Keysym = Keysym(0x1008ff10);
    /// XF86AudioLowerVolume
    pub const KEY_XF86AudioLowerVolume: Keysym = Keysym(0x1008ff11);
    /// XF86AudioMute
    pub const KEY_XF86AudioMute: Keysym = Keysym(0x1008ff12);
    /// XF86AudioRaiseVolume
    pub const KEY_XF86AudioRaiseVolume: Keysym = Keysym(0x1008ff13);
    /// XF86AudioPlay
    pub const KEY_XF86AudioPlay: Keysym = Keysym(0x1008ff14);
    /// XF86AudioStop
    pub const KEY_XF86AudioStop: Keysym = Keysym(0x1008ff15);
    /// XF86AudioPrev
    pub const KEY_XF86AudioPrev: Keysym = Keysym(0x1008ff16);
    /// XF86AudioNext
    pub const KEY_XF86AudioNext: Keysym = Keysym(0x1008ff17);
    /// XF86HomePage
    pub const KEY_XF86HomePage: Keysym = Keysym(0x1008ff18);
    /// XF86Mail
    pub const KEY_XF86Mail: Keysym = Keysym(0x1008ff19);
    /// XF86Start
    pub const KEY_XF86Start: Keysym = Keysym(0x1008ff1a);
    /// XF86Search
    pub const KEY_XF86Search: Keysym = Keysym(0x1008ff1b);
    /// XF86AudioRecord
    pub const KEY_XF86AudioRecord: Keysym = Keysym(0x1008ff1c);
    /// XF86Calculator
    pub const KEY_XF86Calculator: Keysym = Keysym(0x1008ff1d);
    /// XF86Memo
    pub const KEY_XF86Memo: Keysym = Keysym(0x1008ff1e);
    /// XF86ToDoList
    pub const KEY_XF86ToDoList: Keysym = Keysym(0x1008ff1f);
    /// XF86Calendar
    pub const KEY_XF86Calendar: Keysym = Keysym(0x1008ff20);
    /// XF86PowerDown
    pub const KEY_XF86PowerDown: Keysym = Keysym(0x1008ff21);
    /// XF86ContrastAdjust
    pub const KEY_XF86ContrastAdjust: Keysym = Keysym(0x1008ff22);
    /// XF86RockerUp
    pub const KEY_XF86RockerUp: Keysym = Keysym(0x1008ff23);
    /// XF86RockerDown
    pub const KEY_XF86RockerDown: Keysym = Keysym(0x1008ff24);
    /// XF86RockerEnter
    pub const KEY_XF86RockerEnter: Keysym = Keysym(0x1008ff25);
    /// XF86Back
    pub const KEY_XF86Back: Keysym = Keysym(0x1008ff26);
    /// XF86Forward
    pub const KEY_XF86Forward: Keysym = Keysym(0x1008ff27);
    /// XF86Stop
    pub const KEY_XF86Stop: Keysym = Keysym(0x1008ff28);
    /// XF86Refresh
    pub const KEY_XF86Refresh: Keysym = Keysym(0x1008ff29);
    /// XF86PowerOff
    pub const KEY_XF86PowerOff: Keysym = Keysym(0x1008ff2a);
    /// XF86WakeUp
    pub const KEY_XF86WakeUp: Keysym = Keysym(0x1008ff2b);
    /// XF86Eject
    pub const KEY_XF86Eject: Keysym = Keysym(0x1008ff2c);
    /// XF86ScreenSaver
    pub const KEY_XF86ScreenSaver: Keysym = Keysym(0x1008ff2d);
    /// XF86WWW
    pub const KEY_XF86WWW: Keysym = Keysym(0x1008ff2e);
    /// XF86Sleep
    pub const KEY_XF86Sleep: Keysym = Keysym(0x1008ff2f);
    /// XF86Favorites
    pub const KEY_XF86Favorites: Keysym = Keysym(0x1008ff30);
    /// XF86AudioPause
    pub const KEY_XF86AudioPause: Keysym = Keysym(0x1008ff31);
    /// XF86AudioMedia
    pub const KEY_XF86AudioMedia: Keysym = Keysym(0x1008ff32);
    /// XF86MyComputer
    pub const KEY_XF86MyComputer: Keysym = Keysym(0x1008ff33);
    /// XF86VendorHome
    pub const KEY_XF86VendorHome: Keysym = Keysym(0x1008ff34);
    /// XF86LightBulb
    pub const KEY_XF86LightBulb: Keysym = Keysym(0x1008ff35);
    /// XF86Shop
    pub const KEY_XF86Shop: Keysym = Keysym(0x1008ff36);
    /// XF86History
    pub const KEY_XF86History: Keysym = Keysym(0x1008ff37);
    /// XF86OpenURL
    pub const KEY_XF86OpenURL: Keysym = Keysym(0x1008ff38);
    /// XF86AddFavorite
    pub const KEY_XF86AddFavorite: Keysym = Keysym(0x1008ff39);
    /// XF86HotLinks
    pub const KEY_XF86HotLinks: Keysym = Keysym(0x1008ff3a);
    /// XF86BrightnessAdjust
    pub const KEY_XF86BrightnessAdjust: Keysym = Keysym(0x1008ff3b);
    /// XF86Finance
    pub const KEY_XF86Finance: Keysym = Keysym(0x1008ff3c);
    /// XF86Community
    pub const KEY_XF86Community: Keysym = Keysym(0x1008ff3d);
    /// XF86AudioRewind
    pub const KEY_XF86AudioRewind: Keysym = Keysym(0x1008ff3e);
    /// XF86BackForward
    pub const KEY_XF86BackForward: Keysym = Keysym(0x1008ff3f);
    /// XF86Launch0
    pub const KEY_XF86Launch0: Keysym = Keysym(0x1008ff40);
    /// XF86Launch1
    pub const KEY_XF86Launch1: Keysym = Keysym(0x1008ff41);
    /// XF86Launch2
    pub const KEY_XF86Launch2: Keysym = Keysym(0x1008ff42);
    /// XF86Launch3
    pub const KEY_XF86Launch3: Keysym = Keysym(0x1008ff43);
    /// XF86Launch4
    pub const KEY_XF86Launch4: Keysym = Keysym(0x1008ff44);
    /// XF86Launch5
    pub const KEY_XF86Launch5: Keysym = Keysym(0x1008ff45);
    /// XF86Launch6
    pub const KEY_XF86Launch6: Keysym = Keysym(0x1008ff46);
    /// XF86Launch7
    pub const KEY_XF86Launch7: Keysym = Keysym(0x1008ff47);
    /// XF86Launch8
    pub const KEY_XF86Launch8: Keysym = Keysym(0x1008ff48);
    /// XF86Launch9
    pub const KEY_XF86Launch9: Keysym = Keysym(0x1008ff49);
    /// XF86LaunchA
    pub const KEY_XF86LaunchA: Keysym = Keysym(0x1008ff4a);
    /// XF86LaunchB
    pub const KEY_XF86LaunchB: Keysym = Keysym(0x1008ff4b);
    /// XF86LaunchC
    pub const KEY_XF86LaunchC: Keysym = Keysym(0x1008ff4c);
    /// XF86LaunchD
    pub const KEY_XF86LaunchD: Keysym = Keysym(0x1008ff4d);
    /// XF86LaunchE
    pub const KEY_XF86LaunchE: Keysym = Keysym(0x1008ff4e);
    /// XF86LaunchF
    pub const KEY_XF86LaunchF: Keysym = Keysym(0x1008ff4f);
    /// XF86ApplicationLeft
    pub const KEY_XF86ApplicationLeft: Keysym = Keysym(0x1008ff50);
    /// XF86ApplicationRight
    pub const KEY_XF86ApplicationRight: Keysym = Keysym(0x1008ff51);
    /// XF86Book
    pub const KEY_XF86Book: Keysym = Keysym(0x1008ff52);
    /// XF86CD
    pub const KEY_XF86CD: Keysym = Keysym(0x1008ff53);
    /// XF86Calculater
    pub const KEY_XF86Calculater: Keysym = Keysym(0x1008ff54);
    /// XF86Clear
    pub const KEY_XF86Clear: Keysym = Keysym(0x1008ff55);
    /// XF86Close
    pub const KEY_XF86Close: Keysym = Keysym(0x1008ff56);
    /// XF86Copy
    pub const KEY_XF86Copy: Keysym = Keysym(0x1008ff57);
    /// XF86Cut
    pub const KEY_XF86Cut: Keysym = Keysym(0x1008ff58);
    /// XF86Display
    pub const KEY_XF86Display: Keysym = Keysym(0x1008ff59);
    /// XF86DOS
    pub const KEY_XF86DOS: Keysym = Keysym(0x1008ff5a);
    /// XF86Documents
    pub const KEY_XF86Documents: Keysym = Keysym(0x1008ff5b);
    /// XF86Excel
    pub const KEY_XF86Excel: Keysym = Keysym(0x1008ff5c);
    /// XF86Explorer
    pub const KEY_XF86Explorer: Keysym = Keysym(0x1008ff5d);
    /// XF86Game
    pub const KEY_XF86Game: Keysym = Keysym(0x1008ff5e);
    /// XF86Go
    pub const KEY_XF86Go: Keysym = Keysym(0x1008ff5f);
    /// XF86iTouch
    pub const KEY_XF86iTouch: Keysym = Keysym(0x1008ff60);
    /// XF86LogOff
    pub const KEY_XF86LogOff: Keysym = Keysym(0x1008ff61);
    /// XF86Market
    pub const KEY_XF86Market: Keysym = Keysym(0x1008ff62);
    /// XF86Meeting
    pub const KEY_XF86Meeting: Keysym = Keysym(0x1008ff63);
    /// XF86MenuKB
    pub const KEY_XF86MenuKB: Keysym = Keysym(0x1008ff65);
    /// XF86MenuPB
    pub const KEY_XF86MenuPB: Keysym = Keysym(0x1008ff66);
    /// XF86MySites
    pub const KEY_XF86MySites: Keysym = Keysym(0x1008ff67);
    /// XF86New
    pub const KEY_XF86New: Keysym = Keysym(0x1008ff68);
    /// XF86News
    pub const KEY_XF86News: Keysym = Keysym(0x1008ff69);
    /// XF86OfficeHome
    pub const KEY_XF86OfficeHome: Keysym = Keysym(0x1008ff6a);
    /// XF86Open
    pub const KEY_XF86Open: Keysym = Keysym(0x1008ff6b);
    /// XF86Option
    pub const KEY_XF86Option: Keysym = Keysym(0x1008ff6c);
    /// XF86Paste
    pub const KEY_XF86Paste: Keysym = Keysym(0x1008ff6d);
    /// XF86Phone
    pub const KEY_XF86Phone: Keysym = Keysym(0x1008ff6e);
    /// XF86Q
    pub const KEY_XF86Q: Keysym = Keysym(0x1008ff70);
    /// XF86Reply
    pub const KEY_XF86Reply: Keysym = Keysym(0x1008ff72);
    /// XF86Reload
    pub const KEY_XF86Reload: Keysym = Keysym(0x1008ff73);
    /// XF86RotateWindows
    pub const KEY_XF86RotateWindows: Keysym = Keysym(0x1008ff74);
    /// XF86RotationPB
    pub const KEY_XF86RotationPB: Keysym = Keysym(0x1008ff75);
    /// XF86RotationKB
    pub const KEY_XF86RotationKB: Keysym = Keysym(0x1008ff76);
    /// XF86Save
    pub const KEY_XF86Save: Keysym = Keysym(0x1008ff77);
    /// XF86ScrollUp
    pub const KEY_XF86ScrollUp: Keysym = Keysym(0x1008ff78);
    /// XF86ScrollDown
    pub const KEY_XF86ScrollDown: Keysym = Keysym(0x1008ff79);
    /// XF86ScrollClick
    pub const KEY_XF86ScrollClick: Keysym = Keysym(0x1008ff7a);
    /// XF86Send
    pub const KEY_XF86Send: Keysym = Keysym(0x1008ff7b);
    /// XF86Spell
    pub const KEY_XF86Spell: Keysym = Keysym(0x1008ff7c);
    /// XF86SplitScreen
    pub const KEY_XF86SplitScreen: Keysym = Keysym(0x1008ff7d);
    /// XF86Support
    pub const KEY_XF86Support: Keysym = Keysym(0x1008ff7e);
    /// XF86TaskPane
    pub const KEY_XF86TaskPane: Keysym = Keysym(0x1008ff7f);
    /// XF86Terminal
    pub const KEY_XF86Terminal: Keysym = Keysym(0x1008ff80);
    /// XF86Tools
    pub const KEY_XF86Tools: Keysym = Keysym(0x1008ff81);
    /// XF86Travel
    pub const KEY_XF86Travel: Keysym = Keysym(0x1008ff82);
    /// XF86UserPB
    pub const KEY_XF86UserPB: Keysym = Keysym(0x1008ff84);
    /// XF86User1KB
    pub const KEY_XF86User1KB: Keysym = Keysym(0x1008ff85);
    /// XF86User2KB
    pub const KEY_XF86User2KB: Keysym = Keysym(0x1008ff86);
    /// XF86Video
    pub const KEY_XF86Video: Keysym = Keysym(0x1008ff87);
    /// XF86WheelButton
    pub const KEY_XF86WheelButton: Keysym = Keysym(0x1008ff88);
    /// XF86Word
    pub const KEY_XF86Word: Keysym = Keysym(0x1008ff89);
    /// XF86Xfer
    pub const KEY_XF86Xfer: Keysym = Keysym(0x1008ff8a);
    /// XF86ZoomIn
    pub const KEY_XF86ZoomIn: Keysym = Keysym(0x1008ff8b);
    /// XF86ZoomOut
    pub const KEY_XF86ZoomOut: Keysym = Keysym(0x1008ff8c);
    /// XF86Away
    pub const KEY_XF86Away: Keysym = Keysym(0x1008ff8d);
    /// XF86Messenger
    pub const KEY_XF86Messenger: Keysym = Keysym(0x1008ff8e);
    /// XF86WebCam
    pub const KEY_XF86WebCam: Keysym = Keysym(0x1008ff8f);
    /// XF86MailForward
    pub const KEY_XF86MailForward: Keysym = Keysym(0x1008ff90);
    /// XF86Pictures
    pub const KEY_XF86Pictures: Keysym = Keysym(0x1008ff91);
    /// XF86Music
    pub const KEY_XF86Music: Keysym = Keysym(0x1008ff92);
    /// XF86Battery
    pub const KEY_XF86Battery: Keysym = Keysym(0x1008ff93);
    /// XF86Bluetooth
    pub const KEY_XF86Bluetooth: Keysym = Keysym(0x1008ff94);
    /// XF86WLAN
    pub const KEY_XF86WLAN: Keysym = Keysym(0x1008ff95);
    /// XF86UWB
    pub const KEY_XF86UWB: Keysym = Keysym(0x1008ff96);
    /// XF86AudioForward
    pub const KEY_XF86AudioForward: Keysym = Keysym(0x1008ff97);
    /// XF86AudioRepeat
    pub const KEY_XF86AudioRepeat: Keysym = Keysym(0x1008ff98);
    /// XF86AudioRandomPlay
    pub const KEY_XF86AudioRandomPlay: Keysym = Keysym(0x1008ff99);
    /// XF86Subtitle
    pub const KEY_XF86Subtitle: Keysym = Keysym(0x1008ff9a);
    /// XF86AudioCycleTrack
    pub const KEY_XF86AudioCycleTrack: Keysym = Keysym(0x1008ff9b);
    /// XF86CycleAngle
    pub const KEY_XF86CycleAngle: Keysym = Keysym(0x1008ff9c);
    /// XF86FrameBack
    pub const KEY_XF86FrameBack: Keysym = Keysym(0x1008ff9d);
    /// XF86FrameForward
    pub const KEY_XF86FrameForward: Keysym = Keysym(0x1008ff9e);
    /// XF86Time
    pub const KEY_XF86Time: Keysym = Keysym(0x1008ff9f);
    /// XF86Select
    pub const KEY_XF86Select: Keysym = Keysym(0x1008ffa0);
    /// XF86View
    pub const KEY_XF86View: Keysym = Keysym(0x1008ffa1);
    /// XF86TopMenu
    pub const KEY_XF86TopMenu: Keysym = Keysym(0x1008ffa2);
    /// XF86Red
    pub const KEY_XF86Red: Keysym = Keysym(0x1008ffa3);
    /// XF86Green
    pub const KEY_XF86Green: Keysym = Keysym(0x1008ffa4);
    /// XF86Yellow
    pub const KEY_XF86Yellow: Keysym = Keysym(0x1008ffa5);
    /// XF86Blue
    pub const KEY_XF86Blue: Keysym = Keysym(0x1008ffa6);
    /// XF86Suspend
    pub const KEY_XF86Suspend: Keysym = Keysym(0x1008ffa7);
    /// XF86Hibernate
    pub const KEY_XF86Hibernate: Keysym = Keysym(0x1008ffa8);
    /// XF86TouchpadToggle
    pub const KEY_XF86TouchpadToggle: Keysym = Keysym(0x1008ffa9);
    /// XF86TouchpadOn
    pub const KEY_XF86TouchpadOn: Keysym = Keysym(0x1008ffb0);
    /// XF86TouchpadOff
    pub const KEY_XF86TouchpadOff: Keysym = Keysym(0x1008ffb1);
    /// XF86AudioMicMute
    pub const KEY_XF86AudioMicMute: Keysym = Keysym(0x1008ffb2);
    /// XF86Keyboard
    pub const KEY_XF86Keyboard: Keysym = Keysym(0x1008ffb3);
    /// XF86WWAN
    pub const KEY_XF86WWAN: Keysym = Keysym(0x1008ffb4);
    /// XF86RFKill
    pub const KEY_XF86RFKill: Keysym = Keysym(0x1008ffb5);
    /// XF86AudioPreset
    pub const KEY_XF86AudioPreset: Keysym = Keysym(0x1008ffb6);
    /// XF86RotationLockToggle
    pub const KEY_XF86RotationLockToggle: Keysym = Keysym(0x1008ffb7);
    /// XF86FullScreen
    pub const KEY_XF86FullScreen: Keysym = Keysym(0x1008ffb8);
    /// XF86Switch_VT_1
    pub const KEY_XF86Switch_VT_1: Keysym = Keysym(0x1008fe01);
    /// XF86Switch_VT_2
    pub const KEY_XF86Switch_VT_2: Keysym = Keysym(0x1008fe02);
    /// XF86Switch_VT_3
    pub const KEY_XF86Switch_VT_3: Keysym = Keysym(0x1008fe03);
    /// XF86Switch_VT_4
    pub const KEY_XF86Switch_VT_4: Keysym = Keysym(0x1008fe04);
    /// XF86Switch_VT_5
    pub const KEY_XF86Switch_VT_5: Keysym = Keysym(0x1008fe05);
    /// XF86Switch_VT_6
    pub const KEY_XF86Switch_VT_6: Keysym = Keysym(0x1008fe06);
    /// XF86Switch_VT_7
    pub const KEY_XF86Switch_VT_7: Keysym = Keysym(0x1008fe07);
    /// XF86Switch_VT_8
    pub const KEY_XF86Switch_VT_8: Keysym = Keysym(0x1008fe08);
    /// XF86Switch_VT_9
    pub const KEY_XF86Switch_VT_9: Keysym = Keysym(0x1008fe09);
    /// XF86Switch_VT_10
    pub const KEY_XF86Switch_VT_10: Keysym = Keysym(0x1008fe0a);
    /// XF86Switch_VT_11
    pub const KEY_XF86Switch_VT_11: Keysym = Keysym(0x1008fe0b);
    /// XF86Switch_VT_12
    pub const KEY_XF86Switch_VT_12: Keysym = Keysym(0x1008fe0c);
    /// XF86Ungrab
    pub const KEY_XF86Ungrab: Keysym = Keysym(0x1008fe20);
    /// XF86ClearGrab
    pub const KEY_XF86ClearGrab: Keysym = Keysym(0x1008fe21);
    /// XF86Next_VMode
    pub const KEY_XF86Next_VMode: Keysym = Keysym(0x1008fe22);
    /// XF86Prev_VMode
    pub const KEY_XF86Prev_VMode: Keysym = Keysym(0x1008fe23);
    /// XF86LogWindowTree
    pub const KEY_XF86LogWindowTree: Keysym = Keysym(0x1008fe24);
    /// XF86LogGrabInfo
    pub const KEY_XF86LogGrabInfo: Keysym = Keysym(0x1008fe25);
    /// XF86BrightnessAuto
    pub const KEY_XF86BrightnessAuto: Keysym = Keysym(0x100810f4);
    /// XF86DisplayOff
    pub const KEY_XF86DisplayOff: Keysym = Keysym(0x100810f5);
    /// XF86Info
    pub const KEY_XF86Info: Keysym = Keysym(0x10081166);
    /// XF86AspectRatio
    pub const KEY_XF86AspectRatio: Keysym = Keysym(0x10081177);
    /// XF86DVD
    pub const KEY_XF86DVD: Keysym = Keysym(0x10081185);
    /// XF86Audio
    pub const KEY_XF86Audio: Keysym = Keysym(0x10081188);
    /// XF86ChannelUp
    pub const KEY_XF86ChannelUp: Keysym = Keysym(0x10081192);
    /// XF86ChannelDown
    pub const KEY_XF86ChannelDown: Keysym = Keysym(0x10081193);
    /// XF86Break
    pub const KEY_XF86Break: Keysym = Keysym(0x1008119b);
    /// XF86VideoPhone
    pub const KEY_XF86VideoPhone: Keysym = Keysym(0x100811a0);
    /// XF86ZoomReset
    pub const KEY_XF86ZoomReset: Keysym = Keysym(0x100811a4);
    /// XF86Editor
    pub const KEY_XF86Editor: Keysym = Keysym(0x100811a6);
    /// XF86GraphicsEditor
    pub const KEY_XF86GraphicsEditor: Keysym = Keysym(0x100811a8);
    /// XF86Presentation
    pub const KEY_XF86Presentation: Keysym = Keysym(0x100811a9);
    /// XF86Database
    pub const KEY_XF86Database: Keysym = Keysym(0x100811aa);
    /// XF86Voicemail
    pub const KEY_XF86Voicemail: Keysym = Keysym(0x100811ac);
    /// XF86Addressbook
    pub const KEY_XF86Addressbook: Keysym = Keysym(0x100811ad);
    /// XF86DisplayToggle
    pub const KEY_XF86DisplayToggle: Keysym = Keysym(0x100811af);
    /// XF86SpellCheck
    pub const KEY_XF86SpellCheck: Keysym = Keysym(0x100811b0);
    /// XF86ContextMenu
    pub const KEY_XF86ContextMenu: Keysym = Keysym(0x100811b6);
    /// XF86MediaRepeat
    pub const KEY_XF86MediaRepeat: Keysym = Keysym(0x100811b7);
    /// XF8610ChannelsUp
    pub const KEY_XF8610ChannelsUp: Keysym = Keysym(0x100811b8);
    /// XF8610ChannelsDown
    pub const KEY_XF8610ChannelsDown: Keysym = Keysym(0x100811b9);
    /// XF86Images
    pub const KEY_XF86Images: Keysym = Keysym(0x100811ba);
    /// XF86NotificationCenter
    pub const KEY_XF86NotificationCenter: Keysym = Keysym(0x100811bc);
    /// XF86PickupPhone
    pub const KEY_XF86PickupPhone: Keysym = Keysym(0x100811bd);
    /// XF86HangupPhone
    pub const KEY_XF86HangupPhone: Keysym = Keysym(0x100811be);
    /// XF86Fn
    pub const KEY_XF86Fn: Keysym = Keysym(0x100811d0);
    /// XF86Fn_Esc
    pub const KEY_XF86Fn_Esc: Keysym = Keysym(0x100811d1);
    /// XF86FnRightShift
    pub const KEY_XF86FnRightShift: Keysym = Keysym(0x100811e5);
    /// XF86Numeric0
    pub const KEY_XF86Numeric0: Keysym = Keysym(0x10081200);
    /// XF86Numeric1
    pub const KEY_XF86Numeric1: Keysym = Keysym(0x10081201);
    /// XF86Numeric2
    pub const KEY_XF86Numeric2: Keysym = Keysym(0x10081202);
    /// XF86Numeric3
    pub const KEY_XF86Numeric3: Keysym = Keysym(0x10081203);
    /// XF86Numeric4
    pub const KEY_XF86Numeric4: Keysym = Keysym(0x10081204);
    /// XF86Numeric5
    pub const KEY_XF86Numeric5: Keysym = Keysym(0x10081205);
    /// XF86Numeric6
    pub const KEY_XF86Numeric6: Keysym = Keysym(0x10081206);
    /// XF86Numeric7
    pub const KEY_XF86Numeric7: Keysym = Keysym(0x10081207);
    /// XF86Numeric8
    pub const KEY_XF86Numeric8: Keysym = Keysym(0x10081208);
    /// XF86Numeric9
    pub const KEY_XF86Numeric9: Keysym = Keysym(0x10081209);
    /// XF86NumericStar
    pub const KEY_XF86NumericStar: Keysym = Keysym(0x1008120a);
    /// XF86NumericPound
    pub const KEY_XF86NumericPound: Keysym = Keysym(0x1008120b);
    /// XF86NumericA
    pub const KEY_XF86NumericA: Keysym = Keysym(0x1008120c);
    /// XF86NumericB
    pub const KEY_XF86NumericB: Keysym = Keysym(0x1008120d);
    /// XF86NumericC
    pub const KEY_XF86NumericC: Keysym = Keysym(0x1008120e);
    /// XF86NumericD
    pub const KEY_XF86NumericD: Keysym = Keysym(0x1008120f);
    /// XF86CameraFocus
    pub const KEY_XF86CameraFocus: Keysym = Keysym(0x10081210);
    /// XF86WPSButton
    pub const KEY_XF86WPSButton: Keysym = Keysym(0x10081211);
    /// XF86CameraZoomIn
    pub const KEY_XF86CameraZoomIn: Keysym = Keysym(0x10081215);
    /// XF86CameraZoomOut
    pub const KEY_XF86CameraZoomOut: Keysym = Keysym(0x10081216);
    /// XF86CameraUp
    pub const KEY_XF86CameraUp: Keysym = Keysym(0x10081217);
    /// XF86CameraDown
    pub const KEY_XF86CameraDown: Keysym = Keysym(0x10081218);
    /// XF86CameraLeft
    pub const KEY_XF86CameraLeft: Keysym = Keysym(0x10081219);
    /// XF86CameraRight
    pub const KEY_XF86CameraRight: Keysym = Keysym(0x1008121a);
    /// XF86AttendantOn
    pub const KEY_XF86AttendantOn: Keysym = Keysym(0x1008121b);
    /// XF86AttendantOff
    pub const KEY_XF86AttendantOff: Keysym = Keysym(0x1008121c);
    /// XF86AttendantToggle
    pub const KEY_XF86AttendantToggle: Keysym = Keysym(0x1008121d);
    /// XF86LightsToggle
    pub const KEY_XF86LightsToggle: Keysym = Keysym(0x1008121e);
    /// XF86ALSToggle
    pub const KEY_XF86ALSToggle: Keysym = Keysym(0x10081230);
    /// XF86RefreshRateToggle
    pub const KEY_XF86RefreshRateToggle: Keysym = Keysym(0x10081232);
    /// XF86Buttonconfig
    pub const KEY_XF86Buttonconfig: Keysym = Keysym(0x10081240);
    /// XF86Taskmanager
    pub const KEY_XF86Taskmanager: Keysym = Keysym(0x10081241);
    /// XF86Journal
    pub const KEY_XF86Journal: Keysym = Keysym(0x10081242);
    /// XF86ControlPanel
    pub const KEY_XF86ControlPanel: Keysym = Keysym(0x10081243);
    /// XF86AppSelect
    pub const KEY_XF86AppSelect: Keysym = Keysym(0x10081244);
    /// XF86Screensaver
    pub const KEY_XF86Screensaver: Keysym = Keysym(0x10081245);
    /// XF86VoiceCommand
    pub const KEY_XF86VoiceCommand: Keysym = Keysym(0x10081246);
    /// XF86Assistant
    pub const KEY_XF86Assistant: Keysym = Keysym(0x10081247);
    /// XF86EmojiPicker
    pub const KEY_XF86EmojiPicker: Keysym = Keysym(0x10081249);
    /// XF86Dictate
    pub const KEY_XF86Dictate: Keysym = Keysym(0x1008124a);
    /// XF86CameraAccessEnable
    pub const KEY_XF86CameraAccessEnable: Keysym = Keysym(0x1008124b);
    /// XF86CameraAccessDisable
    pub const KEY_XF86CameraAccessDisable: Keysym = Keysym(0x1008124c);
    /// XF86CameraAccessToggle
    pub const KEY_XF86CameraAccessToggle: Keysym = Keysym(0x1008124d);
    /// XF86Accessibility
    pub const KEY_XF86Accessibility: Keysym = Keysym(0x1008124e);
    /// XF86DoNotDisturb
    pub const KEY_XF86DoNotDisturb: Keysym = Keysym(0x1008124f);
    /// XF86BrightnessMin
    pub const KEY_XF86BrightnessMin: Keysym = Keysym(0x10081250);
    /// XF86BrightnessMax
    pub const KEY_XF86BrightnessMax: Keysym = Keysym(0x10081251);
    /// XF86KbdInputAssistPrev
    pub const KEY_XF86KbdInputAssistPrev: Keysym = Keysym(0x10081260);
    /// XF86KbdInputAssistNext
    pub const KEY_XF86KbdInputAssistNext: Keysym = Keysym(0x10081261);
    /// XF86KbdInputAssistPrevgroup
    pub const KEY_XF86KbdInputAssistPrevgroup: Keysym = Keysym(0x10081262);
    /// XF86KbdInputAssistNextgroup
    pub const KEY_XF86KbdInputAssistNextgroup: Keysym = Keysym(0x10081263);
    /// XF86KbdInputAssistAccept
    pub const KEY_XF86KbdInputAssistAccept: Keysym = Keysym(0x10081264);
    /// XF86KbdInputAssistCancel
    pub const KEY_XF86KbdInputAssistCancel: Keysym = Keysym(0x10081265);
    /// XF86RightUp
    pub const KEY_XF86RightUp: Keysym = Keysym(0x10081266);
    /// XF86RightDown
    pub const KEY_XF86RightDown: Keysym = Keysym(0x10081267);
    /// XF86LeftUp
    pub const KEY_XF86LeftUp: Keysym = Keysym(0x10081268);
    /// XF86LeftDown
    pub const KEY_XF86LeftDown: Keysym = Keysym(0x10081269);
    /// XF86RootMenu
    pub const KEY_XF86RootMenu: Keysym = Keysym(0x1008126a);
    /// XF86MediaTopMenu
    pub const KEY_XF86MediaTopMenu: Keysym = Keysym(0x1008126b);
    /// XF86Numeric11
    pub const KEY_XF86Numeric11: Keysym = Keysym(0x1008126c);
    /// XF86Numeric12
    pub const KEY_XF86Numeric12: Keysym = Keysym(0x1008126d);
    /// XF86AudioDesc
    pub const KEY_XF86AudioDesc: Keysym = Keysym(0x1008126e);
    /// XF863DMode
    pub const KEY_XF863DMode: Keysym = Keysym(0x1008126f);
    /// XF86NextFavorite
    pub const KEY_XF86NextFavorite: Keysym = Keysym(0x10081270);
    /// XF86StopRecord
    pub const KEY_XF86StopRecord: Keysym = Keysym(0x10081271);
    /// XF86PauseRecord
    pub const KEY_XF86PauseRecord: Keysym = Keysym(0x10081272);
    /// XF86VOD
    pub const KEY_XF86VOD: Keysym = Keysym(0x10081273);
    /// XF86Unmute
    pub const KEY_XF86Unmute: Keysym = Keysym(0x10081274);
    /// XF86FastReverse
    pub const KEY_XF86FastReverse: Keysym = Keysym(0x10081275);
    /// XF86SlowReverse
    pub const KEY_XF86SlowReverse: Keysym = Keysym(0x10081276);
    /// XF86Data
    pub const KEY_XF86Data: Keysym = Keysym(0x10081277);
    /// XF86OnScreenKeyboard
    pub const KEY_XF86OnScreenKeyboard: Keysym = Keysym(0x10081278);
    /// XF86PrivacyScreenToggle
    pub const KEY_XF86PrivacyScreenToggle: Keysym = Keysym(0x10081279);
    /// XF86SelectiveScreenshot
    pub const KEY_XF86SelectiveScreenshot: Keysym = Keysym(0x1008127a);
    /// XF86NextElement
    pub const KEY_XF86NextElement: Keysym = Keysym(0x1008127b);
    /// XF86PreviousElement
    pub const KEY_XF86PreviousElement: Keysym = Keysym(0x1008127c);
    /// XF86AutopilotEngageToggle
    pub const KEY_XF86AutopilotEngageToggle: Keysym = Keysym(0x1008127d);
    /// XF86MarkWaypoint
    pub const KEY_XF86MarkWaypoint: Keysym = Keysym(0x1008127e);
    /// XF86Sos
    pub const KEY_XF86Sos: Keysym = Keysym(0x1008127f);
    /// XF86NavChart
    pub const KEY_XF86NavChart: Keysym = Keysym(0x10081280);
    /// XF86FishingChart
    pub const KEY_XF86FishingChart: Keysym = Keysym(0x10081281);
    /// XF86SingleRangeRadar
    pub const KEY_XF86SingleRangeRadar: Keysym = Keysym(0x10081282);
    /// XF86DualRangeRadar
    pub const KEY_XF86DualRangeRadar: Keysym = Keysym(0x10081283);
    /// XF86RadarOverlay
    pub const KEY_XF86RadarOverlay: Keysym = Keysym(0x10081284);
    /// XF86TraditionalSonar
    pub const KEY_XF86TraditionalSonar: Keysym = Keysym(0x10081285);
    /// XF86ClearvuSonar
    pub const KEY_XF86ClearvuSonar: Keysym = Keysym(0x10081286);
    /// XF86SidevuSonar
    pub const KEY_XF86SidevuSonar: Keysym = Keysym(0x10081287);
    /// XF86NavInfo
    pub const KEY_XF86NavInfo: Keysym = Keysym(0x10081288);
    /// XF86Macro1
    pub const KEY_XF86Macro1: Keysym = Keysym(0x10081290);
    /// XF86Macro2
    pub const KEY_XF86Macro2: Keysym = Keysym(0x10081291);
    /// XF86Macro3
    pub const KEY_XF86Macro3: Keysym = Keysym(0x10081292);
    /// XF86Macro4
    pub const KEY_XF86Macro4: Keysym = Keysym(0x10081293);
    /// XF86Macro5
    pub const KEY_XF86Macro5: Keysym = Keysym(0x10081294);
    /// XF86Macro6
    pub const KEY_XF86Macro6: Keysym = Keysym(0x10081295);
    /// XF86Macro7
    pub const KEY_XF86Macro7: Keysym = Keysym(0x10081296);
    /// XF86Macro8
    pub const KEY_XF86Macro8: Keysym = Keysym(0x10081297);
    /// XF86Macro9
    pub const KEY_XF86Macro9: Keysym = Keysym(0x10081298);
    /// XF86Macro10
    pub const KEY_XF86Macro10: Keysym = Keysym(0x10081299);
    /// XF86Macro11
    pub const KEY_XF86Macro11: Keysym = Keysym(0x1008129a);
    /// XF86Macro12
    pub const KEY_XF86Macro12: Keysym = Keysym(0x1008129b);
    /// XF86Macro13
    pub const KEY_XF86Macro13: Keysym = Keysym(0x1008129c);
    /// XF86Macro14
    pub const KEY_XF86Macro14: Keysym = Keysym(0x1008129d);
    /// XF86Macro15
    pub const KEY_XF86Macro15: Keysym = Keysym(0x1008129e);
    /// XF86Macro16
    pub const KEY_XF86Macro16: Keysym = Keysym(0x1008129f);
    /// XF86Macro17
    pub const KEY_XF86Macro17: Keysym = Keysym(0x100812a0);
    /// XF86Macro18
    pub const KEY_XF86Macro18: Keysym = Keysym(0x100812a1);
    /// XF86Macro19
    pub const KEY_XF86Macro19: Keysym = Keysym(0x100812a2);
    /// XF86Macro20
    pub const KEY_XF86Macro20: Keysym = Keysym(0x100812a3);
    /// XF86Macro21
    pub const KEY_XF86Macro21: Keysym = Keysym(0x100812a4);
    /// XF86Macro22
    pub const KEY_XF86Macro22: Keysym = Keysym(0x100812a5);
    /// XF86Macro23
    pub const KEY_XF86Macro23: Keysym = Keysym(0x100812a6);
    /// XF86Macro24
    pub const KEY_XF86Macro24: Keysym = Keysym(0x100812a7);
    /// XF86Macro25
    pub const KEY_XF86Macro25: Keysym = Keysym(0x100812a8);
    /// XF86Macro26
    pub const KEY_XF86Macro26: Keysym = Keysym(0x100812a9);
    /// XF86Macro27
    pub const KEY_XF86Macro27: Keysym = Keysym(0x100812aa);
    /// XF86Macro28
    pub const KEY_XF86Macro28: Keysym = Keysym(0x100812ab);
    /// XF86Macro29
    pub const KEY_XF86Macro29: Keysym = Keysym(0x100812ac);
    /// XF86Macro30
    pub const KEY_XF86Macro30: Keysym = Keysym(0x100812ad);
    /// XF86MacroRecordStart
    pub const KEY_XF86MacroRecordStart: Keysym = Keysym(0x100812b0);
    /// XF86MacroRecordStop
    pub const KEY_XF86MacroRecordStop: Keysym = Keysym(0x100812b1);
    /// XF86MacroPresetCycle
    pub const KEY_XF86MacroPresetCycle: Keysym = Keysym(0x100812b2);
    /// XF86MacroPreset1
    pub const KEY_XF86MacroPreset1: Keysym = Keysym(0x100812b3);
    /// XF86MacroPreset2
    pub const KEY_XF86MacroPreset2: Keysym = Keysym(0x100812b4);
    /// XF86MacroPreset3
    pub const KEY_XF86MacroPreset3: Keysym = Keysym(0x100812b5);
    /// XF86KbdLcdMenu1
    pub const KEY_XF86KbdLcdMenu1: Keysym = Keysym(0x100812b8);
    /// XF86KbdLcdMenu2
    pub const KEY_XF86KbdLcdMenu2: Keysym = Keysym(0x100812b9);
    /// XF86KbdLcdMenu3
    pub const KEY_XF86KbdLcdMenu3: Keysym = Keysym(0x100812ba);
    /// XF86KbdLcdMenu4
    pub const KEY_XF86KbdLcdMenu4: Keysym = Keysym(0x100812bb);
    /// XF86KbdLcdMenu5
    pub const KEY_XF86KbdLcdMenu5: Keysym = Keysym(0x100812bc);
    /// SunFA_Grave
    pub const KEY_SunFA_Grave: Keysym = Keysym(0x1005ff00);
    /// SunFA_Circum
    pub const KEY_SunFA_Circum: Keysym = Keysym(0x1005ff01);
    /// SunFA_Tilde
    pub const KEY_SunFA_Tilde: Keysym = Keysym(0x1005ff02);
    /// SunFA_Acute
    pub const KEY_SunFA_Acute: Keysym = Keysym(0x1005ff03);
    /// SunFA_Diaeresis
    pub const KEY_SunFA_Diaeresis: Keysym = Keysym(0x1005ff04);
    /// SunFA_Cedilla
    pub const KEY_SunFA_Cedilla: Keysym = Keysym(0x1005ff05);
    /// SunF36
    pub const KEY_SunF36: Keysym = Keysym(0x1005ff10);
    /// SunF37
    pub const KEY_SunF37: Keysym = Keysym(0x1005ff11);
    /// SunSys_Req
    pub const KEY_SunSys_Req: Keysym = Keysym(0x1005ff60);
    /// SunProps
    pub const KEY_SunProps: Keysym = Keysym(0x1005ff70);
    /// SunFront
    pub const KEY_SunFront: Keysym = Keysym(0x1005ff71);
    /// SunCopy
    pub const KEY_SunCopy: Keysym = Keysym(0x1005ff72);
    /// SunOpen
    pub const KEY_SunOpen: Keysym = Keysym(0x1005ff73);
    /// SunPaste
    pub const KEY_SunPaste: Keysym = Keysym(0x1005ff74);
    /// SunCut
    pub const KEY_SunCut: Keysym = Keysym(0x1005ff75);
    /// SunPowerSwitch
    pub const KEY_SunPowerSwitch: Keysym = Keysym(0x1005ff76);
    /// SunAudioLowerVolume
    pub const KEY_SunAudioLowerVolume: Keysym = Keysym(0x1005ff77);
    /// SunAudioMute
    pub const KEY_SunAudioMute: Keysym = Keysym(0x1005ff78);
    /// SunAudioRaiseVolume
    pub const KEY_SunAudioRaiseVolume: Keysym = Keysym(0x1005ff79);
    /// SunVideoDegauss
    pub const KEY_SunVideoDegauss: Keysym = Keysym(0x1005ff7a);
    /// SunVideoLowerBrightness
    pub const KEY_SunVideoLowerBrightness: Keysym = Keysym(0x1005ff7b);
    /// SunVideoRaiseBrightness
    pub const KEY_SunVideoRaiseBrightness: Keysym = Keysym(0x1005ff7c);
    /// SunPowerSwitchShift
    pub const KEY_SunPowerSwitchShift: Keysym = Keysym(0x1005ff7d);
    /// Dring_accent
    pub const KEY_Dring_accent: Keysym = Keysym(0x1000feb0);
    /// Dcircumflex_accent
    pub const KEY_Dcircumflex_accent: Keysym = Keysym(0x1000fe5e);
    /// Dcedilla_accent
    pub const KEY_Dcedilla_accent: Keysym = Keysym(0x1000fe2c);
    /// Dacute_accent
    pub const KEY_Dacute_accent: Keysym = Keysym(0x1000fe27);
    /// Dgrave_accent
    pub const KEY_Dgrave_accent: Keysym = Keysym(0x1000fe60);
    /// Dtilde
    pub const KEY_Dtilde: Keysym = Keysym(0x1000fe7e);
    /// Ddiaeresis
    pub const KEY_Ddiaeresis: Keysym = Keysym(0x1000fe22);
    /// DRemove
    pub const KEY_DRemove: Keysym = Keysym(0x1000ff00);
    /// hpClearLine
    pub const KEY_hpClearLine: Keysym = Keysym(0x1000ff6f);
    /// ClearLine
    pub const KEY_ClearLine: Keysym = Keysym(0x1000ff6f);
    /// hpInsertLine
    pub const KEY_hpInsertLine: Keysym = Keysym(0x1000ff70);
    /// InsertLine
    pub const KEY_InsertLine: Keysym = Keysym(0x1000ff70);
    /// hpDeleteLine
    pub const KEY_hpDeleteLine: Keysym = Keysym(0x1000ff71);
    /// DeleteLine
    pub const KEY_DeleteLine: Keysym = Keysym(0x1000ff71);
    /// hpInsertChar
    pub const KEY_hpInsertChar: Keysym = Keysym(0x1000ff72);
    /// InsertChar
    pub const KEY_InsertChar: Keysym = Keysym(0x1000ff72);
    /// hpDeleteChar
    pub const KEY_hpDeleteChar: Keysym = Keysym(0x1000ff73);
    /// DeleteChar
    pub const KEY_DeleteChar: Keysym = Keysym(0x1000ff73);
    /// hpBackTab
    pub const KEY_hpBackTab: Keysym = Keysym(0x1000ff74);
    /// BackTab
    pub const KEY_BackTab: Keysym = Keysym(0x1000ff74);
    /// hpKP_BackTab
    pub const KEY_hpKP_BackTab: Keysym = Keysym(0x1000ff75);
    /// KP_BackTab
    pub const KEY_KP_BackTab: Keysym = Keysym(0x1000ff75);
    /// hpModelock1
    pub const KEY_hpModelock1: Keysym = Keysym(0x1000ff48);
    /// hpModelock2
    pub const KEY_hpModelock2: Keysym = Keysym(0x1000ff49);
    /// hpReset
    pub const KEY_hpReset: Keysym = Keysym(0x1000ff6c);
    /// Reset
    pub const KEY_Reset: Keysym = Keysym(0x1000ff6c);
    /// hpSystem
    pub const KEY_hpSystem: Keysym = Keysym(0x1000ff6d);
    /// System
    pub const KEY_System: Keysym = Keysym(0x1000ff6d);
    /// hpUser
    pub const KEY_hpUser: Keysym = Keysym(0x1000ff6e);
    /// User
    pub const KEY_User: Keysym = Keysym(0x1000ff6e);
    /// hpmute_acute
    pub const KEY_hpmute_acute: Keysym = Keysym(0x100000a8);
    /// mute_acute
    pub const KEY_mute_acute: Keysym = Keysym(0x100000a8);
    /// hpmute_grave
    pub const KEY_hpmute_grave: Keysym = Keysym(0x100000a9);
    /// mute_grave
    pub const KEY_mute_grave: Keysym = Keysym(0x100000a9);
    /// hpmute_asciicircum
    pub const KEY_hpmute_asciicircum: Keysym = Keysym(0x100000aa);
    /// mute_asciicircum
    pub const KEY_mute_asciicircum: Keysym = Keysym(0x100000aa);
    /// hpmute_diaeresis
    pub const KEY_hpmute_diaeresis: Keysym = Keysym(0x100000ab);
    /// mute_diaeresis
    pub const KEY_mute_diaeresis: Keysym = Keysym(0x100000ab);
    /// hpmute_asciitilde
    pub const KEY_hpmute_asciitilde: Keysym = Keysym(0x100000ac);
    /// mute_asciitilde
    pub const KEY_mute_asciitilde: Keysym = Keysym(0x100000ac);
    /// hplira
    pub const KEY_hplira: Keysym = Keysym(0x100000af);
    /// lira
    pub const KEY_lira: Keysym = Keysym(0x100000af);
    /// hpguilder
    pub const KEY_hpguilder: Keysym = Keysym(0x100000be);
    /// guilder
    pub const KEY_guilder: Keysym = Keysym(0x100000be);
    /// hpYdiaeresis
    pub const KEY_hpYdiaeresis: Keysym = Keysym(0x100000ee);
    /// hpIO
    pub const KEY_hpIO: Keysym = Keysym(0x100000ee);
    /// IO
    pub const KEY_IO: Keysym = Keysym(0x100000ee);
    /// hplongminus
    pub const KEY_hplongminus: Keysym = Keysym(0x100000f6);
    /// longminus
    pub const KEY_longminus: Keysym = Keysym(0x100000f6);
    /// hpblock
    pub const KEY_hpblock: Keysym = Keysym(0x100000fc);
    /// block
    pub const KEY_block: Keysym = Keysym(0x100000fc);
    /// osfCopy
    pub const KEY_osfCopy: Keysym = Keysym(0x1004ff02);
    /// osfCut
    pub const KEY_osfCut: Keysym = Keysym(0x1004ff03);
    /// osfPaste
    pub const KEY_osfPaste: Keysym = Keysym(0x1004ff04);
    /// osfBackTab
    pub const KEY_osfBackTab: Keysym = Keysym(0x1004ff07);
    /// osfBackSpace
    pub const KEY_osfBackSpace: Keysym = Keysym(0x1004ff08);
    /// osfClear
    pub const KEY_osfClear: Keysym = Keysym(0x1004ff0b);
    /// osfEscape
    pub const KEY_osfEscape: Keysym = Keysym(0x1004ff1b);
    /// osfAddMode
    pub const KEY_osfAddMode: Keysym = Keysym(0x1004ff31);
    /// osfPrimaryPaste
    pub const KEY_osfPrimaryPaste: Keysym = Keysym(0x1004ff32);
    /// osfQuickPaste
    pub const KEY_osfQuickPaste: Keysym = Keysym(0x1004ff33);
    /// osfPageLeft
    pub const KEY_osfPageLeft: Keysym = Keysym(0x1004ff40);
    /// osfPageUp
    pub const KEY_osfPageUp: Keysym = Keysym(0x1004ff41);
    /// osfPageDown
    pub const KEY_osfPageDown: Keysym = Keysym(0x1004ff42);
    /// osfPageRight
    pub const KEY_osfPageRight: Keysym = Keysym(0x1004ff43);
    /// osfActivate
    pub const KEY_osfActivate: Keysym = Keysym(0x1004ff44);
    /// osfMenuBar
    pub const KEY_osfMenuBar: Keysym = Keysym(0x1004ff45);
    /// osfLeft
    pub const KEY_osfLeft: Keysym = Keysym(0x1004ff51);
    /// osfUp
    pub const KEY_osfUp: Keysym = Keysym(0x1004ff52);
    /// osfRight
    pub const KEY_osfRight: Keysym = Keysym(0x1004ff53);
    /// osfDown
    pub const KEY_osfDown: Keysym = Keysym(0x1004ff54);
    /// osfEndLine
    pub const KEY_osfEndLine: Keysym = Keysym(0x1004ff57);
    /// osfBeginLine
    pub const KEY_osfBeginLine: Keysym = Keysym(0x1004ff58);
    /// osfEndData
    pub const KEY_osfEndData: Keysym = Keysym(0x1004ff59);
    /// osfBeginData
    pub const KEY_osfBeginData: Keysym = Keysym(0x1004ff5a);
    /// osfPrevMenu
    pub const KEY_osfPrevMenu: Keysym = Keysym(0x1004ff5b);
    /// osfNextMenu
    pub const KEY_osfNextMenu: Keysym = Keysym(0x1004ff5c);
    /// osfPrevField
    pub const KEY_osfPrevField: Keysym = Keysym(0x1004ff5d);
    /// osfNextField
    pub const KEY_osfNextField: Keysym = Keysym(0x1004ff5e);
    /// osfSelect
    pub const KEY_osfSelect: Keysym = Keysym(0x1004ff60);
    /// osfInsert
    pub const KEY_osfInsert: Keysym = Keysym(0x1004ff63);
    /// osfUndo
    pub const KEY_osfUndo: Keysym = Keysym(0x1004ff65);
    /// osfMenu
    pub const KEY_osfMenu: Keysym = Keysym(0x1004ff67);
    /// osfCancel
    pub const KEY_osfCancel: Keysym = Keysym(0x1004ff69);
    /// osfHelp
    pub const KEY_osfHelp: Keysym = Keysym(0x1004ff6a);
    /// osfSelectAll
    pub const KEY_osfSelectAll: Keysym = Keysym(0x1004ff71);
    /// osfDeselectAll
    pub const KEY_osfDeselectAll: Keysym = Keysym(0x1004ff72);
    /// osfReselect
    pub const KEY_osfReselect: Keysym = Keysym(0x1004ff73);
    /// osfExtend
    pub const KEY_osfExtend: Keysym = Keysym(0x1004ff74);
    /// osfRestore
    pub const KEY_osfRestore: Keysym = Keysym(0x1004ff78);
    /// osfDelete
    pub const KEY_osfDelete: Keysym = Keysym(0x1004ffff);
    /// Ext16bit_L
    pub const KEY_Ext16bit_L: Keysym = Keysym(0x1000ff76);
    /// Ext16bit_R
    pub const KEY_Ext16bit_R: Keysym = Keysym(0x1000ff77);
}

