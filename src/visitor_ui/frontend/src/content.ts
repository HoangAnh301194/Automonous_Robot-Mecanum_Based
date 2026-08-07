export type Language = 'vi' | 'en';

export type Place = {
  id: string;
  name: Record<Language, string>;
  category: Record<Language, string>;
  description: Record<Language, string>;
  x: number;
  y: number;
  yaw: number;
};

export type KnowledgeEntry = {
  id: string;
  title: Record<Language, string>;
  answer: Record<Language, string>;
  keywords: string[];
  placeId?: string;
};

export const places: Place[] = [
  {
    id: 'reception',
    name: { vi: 'Quay le tan', en: 'Reception' },
    category: { vi: 'Dich vu', en: 'Service' },
    description: { vi: 'Noi ho tro thong tin va tiep nhan khach.', en: 'Visitor information and assistance desk.' },
    x: 0.50, y: 0.50, yaw: 0.0,
  },
  {
    id: 'cafeteria',
    name: { vi: 'Can tin', en: 'Cafeteria' },
    category: { vi: 'Tien ich', en: 'Facility' },
    description: { vi: 'Khu vuc an uong.', en: 'Dining area.' },
    x: 2.50, y: 1.50, yaw: 1.57,
  },
  {
    id: 'restroom',
    name: { vi: 'Nha ve sinh', en: 'Restroom' },
    category: { vi: 'Tien ich', en: 'Facility' },
    description: { vi: 'Nha ve sinh gan nhat.', en: 'Nearest restroom.' },
    x: 1.50, y: 0.50, yaw: 0.0,
  },
  {
    id: 'library',
    name: { vi: 'Thu vien', en: 'Library' },
    category: { vi: 'Hoc tap', en: 'Study' },
    description: { vi: 'Thu vien truong hoc.', en: 'School library.' },
    x: 3.00, y: 2.00, yaw: 0.0,
  },
  {
    id: 'admin_office',
    name: { vi: 'Phong hanh chinh', en: 'Admin Office' },
    category: { vi: 'Dich vu', en: 'Service' },
    description: { vi: 'Van phong hanh chinh.', en: 'Administrative office.' },
    x: 1.00, y: 2.50, yaw: 3.14,
  },
];

export const knowledgeEntries: KnowledgeEntry[] = [
  {
    id: 'robot_intro',
    title: { vi: 'Gioi thieu robot', en: 'About this robot' },
    answer: {
      vi: 'Robot ho tro khach tim duong, cung cap thong tin va tiep nhan phan hoi tai sanh.',
      en: 'This robot helps visitors find directions, provides information, and collects feedback at the lobby.',
    },
    keywords: ['robot', 'gioi thieu', 'la gi', 'introduce', 'about'],
  },
  {
    id: 'how_to_navigate',
    title: { vi: 'Cach chon diem den', en: 'How to navigate' },
    answer: {
      vi: 'Chon Dan duong, chon mot dia diem trong danh sach hoac cham ban do, roi xac nhan.',
      en: 'Select Navigation, choose a location from the list or tap the map, then confirm.',
    },
    keywords: ['cach', 'dan duong', 'di den', 'navigate', 'how', 'direction'],
  },
  {
    id: 'reception_info',
    title: { vi: 'Quay le tan o dau?', en: 'Where is the reception?' },
    answer: {
      vi: 'Toi co the dan ban den quay le tan.',
      en: 'I can guide you to the reception desk.',
    },
    keywords: ['le tan', 'reception', 'o dau', 'where'],
    placeId: 'reception',
  },
  {
    id: 'cafeteria_info',
    title: { vi: 'Can tin o dau?', en: 'Where is the cafeteria?' },
    answer: {
      vi: 'Can tin nam o tang 1. Toi co the dan ban den.',
      en: 'The cafeteria is on the 1st floor. I can guide you there.',
    },
    keywords: ['can tin', 'cafeteria', 'an', 'uong', 'food'],
    placeId: 'cafeteria',
  },
  {
    id: 'restroom_info',
    title: { vi: 'Nha ve sinh o dau?', en: 'Where is the restroom?' },
    answer: {
      vi: 'Nha ve sinh gan nhat o cuoi hanh lang ben phai.',
      en: 'The nearest restroom is at the end of the right hallway.',
    },
    keywords: ['ve sinh', 'restroom', 'toilet', 'wc'],
    placeId: 'restroom',
  },
  {
    id: 'privacy',
    title: { vi: 'Quyen rieng tu', en: 'Privacy policy' },
    answer: {
      vi: 'Giao dien khach khong luu lich su cau hoi hoac du lieu ca nhan. Camera preview khong hien thi mac dinh.',
      en: 'The visitor interface does not store question history or personal data. Camera preview is hidden by default.',
    },
    keywords: ['quyen', 'rieng tu', 'privacy', 'camera', 'du lieu'],
  },
  {
    id: 'working_hours',
    title: { vi: 'Gio lam viec', en: 'Working hours' },
    answer: {
      vi: 'Truong hoat dong tu 7:00 den 17:00, thu Hai den thu Sau.',
      en: 'The school operates from 7:00 AM to 5:00 PM, Monday to Friday.',
    },
    keywords: ['gio', 'lam viec', 'working hours', 'mo cua', 'open'],
  },
  {
    id: 'contact',
    title: { vi: 'Thong tin lien he', en: 'Contact information' },
    answer: {
      vi: 'Vui long lien he quay le tan hoac goi so: (024) xxxx-xxxx.',
      en: 'Please contact the reception desk or call: (024) xxxx-xxxx.',
    },
    keywords: ['lien he', 'contact', 'so dien thoai', 'phone', 'email'],
  },
];

export const labels = {
  vi: {
    home: 'Trang chu',
    navigation: 'Dan duong',
    information: 'Hoi dap',
    settings: 'Cai dat',
    feedback: 'Phan hoi',
    welcome_eyebrow: 'ROBOT HO TRO SANH',
    welcome_title: 'Xin chao!',
    welcome_sub: 'Toi co the ho tro gi cho ban?',
    touch_hint: 'Cham vao mot lua chon de bat dau',
    select_dest: 'Chon diem den',
    confirm: 'Xac nhan',
    cancel: 'Huy',
    start_nav: 'Bat dau di',
    navigating_text: 'Robot dang di chuyen...',
    arrived_text: 'Da den noi!',
    search_placeholder: 'Nhap cau hoi...',
    guide_me: 'Dan toi den',
    no_answer: 'Toi chua co thong tin nay. Vui long lien he le tan.',
    language_label: 'Ngon ngu',
    font_size: 'Co chu',
    high_contrast: 'Do tuong phan cao',
    about_robot: 'Thong tin robot',
    privacy_label: 'Quyen rieng tu',
    rate_label: 'Danh gia trai nghiem',
    category_label: 'Chu de',
    comment_label: 'Gop y (tuy chon)',
    submit: 'Gui phan hoi',
    thank_title: 'Cam on ban!',
    thank_sub: 'Phan hoi cua ban giup robot cai thien hon.',
    rating_great: 'Rat tot',
    rating_good: 'Tot',
    rating_ok: 'Binh thuong',
    rating_bad: 'Chua tot',
    cat_nav: 'Dan duong',
    cat_info: 'Thong tin',
    cat_ui: 'Giao dien',
    cat_other: 'Khac',
    map_placeholder: 'Ban do se hien thi khi ket noi ROS',
  },
  en: {
    home: 'Home',
    navigation: 'Navigation',
    information: 'Q & A',
    settings: 'Settings',
    feedback: 'Feedback',
    welcome_eyebrow: 'LOBBY ASSISTANT ROBOT',
    welcome_title: 'Hello!',
    welcome_sub: 'How can I help you?',
    touch_hint: 'Tap an option to get started',
    select_dest: 'Select destination',
    confirm: 'Confirm',
    cancel: 'Cancel',
    start_nav: 'Start',
    navigating_text: 'Robot is moving...',
    arrived_text: 'Arrived!',
    search_placeholder: 'Type your question...',
    guide_me: 'Guide me there',
    no_answer: 'I do not have this information yet. Please contact the reception.',
    language_label: 'Language',
    font_size: 'Font size',
    high_contrast: 'High contrast',
    about_robot: 'About robot',
    privacy_label: 'Privacy',
    rate_label: 'Rate your experience',
    category_label: 'Category',
    comment_label: 'Comment (optional)',
    submit: 'Submit feedback',
    thank_title: 'Thank you!',
    thank_sub: 'Your feedback helps the robot improve.',
    rating_great: 'Great',
    rating_good: 'Good',
    rating_ok: 'OK',
    rating_bad: 'Poor',
    cat_nav: 'Navigation',
    cat_info: 'Information',
    cat_ui: 'Interface',
    cat_other: 'Other',
    map_placeholder: 'Map will appear when connected to ROS',
  },
};
