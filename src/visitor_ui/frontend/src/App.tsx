import { createElement, useEffect, useMemo, useState, useCallback } from 'react';
import { places, knowledgeEntries, labels, type Language, type Place, type KnowledgeEntry } from './content';

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */
type Screen = 'home' | 'navigation' | 'information' | 'settings' | 'feedback';
type EmotionState = 'idle' | 'greeting' | 'thinking' | 'navigating' | 'arrived' | 'warning' | 'offline';
type NavPhase = 'select' | 'confirm' | 'moving' | 'arrived';
type Rating = 'great' | 'good' | 'ok' | 'bad' | null;
type FeedbackCategory = 'nav' | 'info' | 'ui' | 'other' | null;

const SESSION_TIMEOUT_MS = 60_000;

/* ------------------------------------------------------------------ */
/* App                                                                 */
/* ------------------------------------------------------------------ */
export default function App() {
  const [screen, setScreen] = useState<Screen>('home');
  const [emotion, setEmotion] = useState<EmotionState>('idle');
  const [language, setLanguage] = useState<Language>('vi');
  const [highContrast, setHighContrast] = useState(false);
  const [fontScale, setFontScale] = useState(1);

  const t = useMemo(() => labels[language], [language]);

  // Session idle reset
  useEffect(() => {
    const timer = window.setTimeout(() => {
      setScreen('home');
      setEmotion('idle');
    }, SESSION_TIMEOUT_MS);
    return () => window.clearTimeout(timer);
  }, [screen, language]);

  // Apply font-scale + high-contrast
  useEffect(() => {
    document.documentElement.style.setProperty('--font-scale', String(fontScale));
    if (highContrast) {
      document.documentElement.classList.add('high-contrast');
    } else {
      document.documentElement.classList.remove('high-contrast');
    }
  }, [fontScale, highContrast]);

  const open = useCallback((next: Screen) => {
    setEmotion('greeting');
    setScreen(next);
  }, []);

  const goHome = useCallback(() => {
    setScreen('home');
    setEmotion('idle');
  }, []);

  const toggleLang = useCallback(() => {
    setLanguage(prev => (prev === 'vi' ? 'en' : 'vi'));
  }, []);

  return (
    <div className="visitor-app">
      <header className="topbar">
        <div className="brand-mark">
          <span className="brand-dot"></span> LOBBY ROBOT
        </div>
        <button className="language-toggle" onClick={toggleLang}>
          {language.toUpperCase()}
        </button>
      </header>

      <main className="screen-area">
        {screen === 'home' && (
          <HomeView emotion={emotion} open={open} t={t} />
        )}
        {screen === 'navigation' && (
          <NavigationView
            language={language}
            setEmotion={setEmotion}
            goHome={goHome}
            t={t}
          />
        )}
        {screen === 'information' && (
          <InformationView
            language={language}
            setEmotion={setEmotion}
            t={t}
            openNav={() => setScreen('navigation')}
          />
        )}
        {screen === 'settings' && (
          <SettingsView
            language={language}
            setLanguage={setLanguage}
            fontScale={fontScale}
            setFontScale={setFontScale}
            highContrast={highContrast}
            setHighContrast={setHighContrast}
            t={t}
          />
        )}
        {screen === 'feedback' && (
          <FeedbackView
            setEmotion={setEmotion}
            goHome={goHome}
            t={t}
          />
        )}
      </main>

      {screen !== 'home' && (
        <button className="home-button" onClick={goHome}>
          {t.home}
        </button>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* EmotionFace                                                         */
/* ------------------------------------------------------------------ */
function EmotionFace({ state }: { state: EmotionState }) {
  return createElement(
    'div',
    { className: 'emotion-wrap', 'data-state': state },
    createElement('div', { className: 'emotion-face' },
      createElement('div', { className: 'emotion-mouth' })
    )
  );
}

/* ------------------------------------------------------------------ */
/* HomeView                                                            */
/* ------------------------------------------------------------------ */
function HomeView({ emotion, open, t }: {
  emotion: EmotionState;
  open: (s: Screen) => void;
  t: typeof labels['vi'];
}) {
  return (
    <section className="home-screen">
      <div className="welcome-copy">
        <p className="eyebrow">{t.welcome_eyebrow}</p>
        <h1>{t.welcome_title}</h1>
        <p>{t.welcome_sub}</p>
      </div>
      <EmotionFace state={emotion} />
      <p className="touch-hint">{t.touch_hint}</p>
      <HomeMenu open={open} t={t} />
    </section>
  );
}

/* ------------------------------------------------------------------ */
/* HomeMenu                                                            */
/* ------------------------------------------------------------------ */
function HomeMenu({ open, t }: { open: (s: Screen) => void; t: typeof labels['vi'] }) {
  const items: { screen: Screen; icon: string; label: string }[] = [
    { screen: 'navigation', icon: 'NAV', label: t.navigation },
    { screen: 'information', icon: '?', label: t.information },
    { screen: 'settings', icon: 'SET', label: t.settings },
    { screen: 'feedback', icon: 'FB', label: t.feedback },
  ];

  return (
    <div className="home-menu">
      {items.map(item => (
        <button
          key={item.screen}
          className="menu-card"
          onClick={() => open(item.screen)}
        >
          <span className="menu-icon">{item.icon}</span>
          <strong>{item.label}</strong>
        </button>
      ))}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* NavigationView                                                      */
/* ------------------------------------------------------------------ */
function NavigationView({ language, setEmotion, goHome, t }: {
  language: Language;
  setEmotion: (e: EmotionState) => void;
  goHome: () => void;
  t: typeof labels['vi'];
}) {
  const [phase, setPhase] = useState<NavPhase>('select');
  const [selected, setSelected] = useState<Place | null>(null);

  const handleSelect = (place: Place) => {
    setSelected(place);
    setPhase('confirm');
    setEmotion('thinking');
  };

  const handleStart = () => {
    setPhase('moving');
    setEmotion('navigating');
    // Mock: arrive after 5s
    window.setTimeout(() => {
      setPhase('arrived');
      setEmotion('arrived');
    }, 5000);
  };

  const handleCancel = () => {
    setSelected(null);
    setPhase('select');
    setEmotion('greeting');
  };

  if (phase === 'arrived') {
    return (
      <div className="thank-you">
        <EmotionFace state="arrived" />
        <h2>{t.arrived_text}</h2>
        <p>{selected ? selected.name[language] : ''}</p>
        <button className="btn-primary" onClick={goHome}>{t.home}</button>
      </div>
    );
  }

  if (phase === 'moving') {
    return (
      <div className="nav-progress">
        <EmotionFace state="navigating" />
        <p className="progress-text">{t.navigating_text}</p>
        <div style={{ marginTop: '24px' }}>
          <button className="btn-secondary" onClick={handleCancel}>{t.cancel}</button>
        </div>
      </div>
    );
  }

  return (
    <div className="navigation-view">
      <h2 className="nav-title">{t.select_dest}</h2>

      <div className="poi-list">
        {places.map(place => (
          <button
            key={place.id}
            className={'poi-card' + (selected?.id === place.id ? ' selected' : '')}
            onClick={() => handleSelect(place)}
          >
            <span className="poi-name">{place.name[language]}</span>
            <span className="poi-cat">{place.category[language]}</span>
            <span className="poi-desc">{place.description[language]}</span>
          </button>
        ))}
      </div>

      <div className="map-mock">
        <span>{t.map_placeholder}</span>
      </div>

      {phase === 'confirm' && selected && (
        <div className="nav-confirm-bar">
          <button className="btn-primary" onClick={handleStart}>
            {t.start_nav}: {selected.name[language]}
          </button>
          <button className="btn-secondary" onClick={handleCancel}>
            {t.cancel}
          </button>
        </div>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* InformationView                                                     */
/* ------------------------------------------------------------------ */
function InformationView({ language, setEmotion, t, openNav }: {
  language: Language;
  setEmotion: (e: EmotionState) => void;
  t: typeof labels['vi'];
  openNav: () => void;
}) {
  const [query, setQuery] = useState('');
  const [activeEntry, setActiveEntry] = useState<KnowledgeEntry | null>(null);

  const searchResults = useMemo(() => {
    if (!query.trim()) return [];
    const q = query.toLowerCase();
    return knowledgeEntries.filter(entry =>
      entry.keywords.some(kw => kw.includes(q) || q.includes(kw)) ||
      entry.title[language].toLowerCase().includes(q)
    );
  }, [query, language]);

  const handleSelectEntry = (entry: KnowledgeEntry) => {
    setActiveEntry(entry);
    setEmotion('thinking');
    window.setTimeout(() => setEmotion('greeting'), 600);
  };

  const handleClear = () => {
    setActiveEntry(null);
    setQuery('');
  };

  return (
    <div className="info-view">
      <h2 className="info-title">{t.information}</h2>

      <div className="search-box">
        <input
          className="search-input"
          placeholder={t.search_placeholder}
          value={query}
          onChange={e => { setQuery(e.target.value); setActiveEntry(null); }}
        />
      </div>

      {activeEntry ? (
        <div className="answer-panel">
          <h3>{activeEntry.title[language]}</h3>
          <p>{activeEntry.answer[language]}</p>
          <div className="answer-actions">
            {activeEntry.placeId && (
              <button className="btn-primary" onClick={openNav}>
                {t.guide_me}
              </button>
            )}
            <button className="btn-secondary" onClick={handleClear}>
              {t.cancel}
            </button>
          </div>
        </div>
      ) : query.trim() && searchResults.length === 0 ? (
        <div className="answer-panel">
          <p>{t.no_answer}</p>
        </div>
      ) : null}

      <div className="quick-questions">
        {(query.trim() && searchResults.length > 0 ? searchResults : knowledgeEntries).map(entry => (
          <button
            key={entry.id}
            className="quick-card"
            onClick={() => handleSelectEntry(entry)}
          >
            <span className="quick-title">{entry.title[language]}</span>
          </button>
        ))}
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* SettingsView                                                        */
/* ------------------------------------------------------------------ */
function SettingsView({ language, setLanguage, fontScale, setFontScale, highContrast, setHighContrast, t }: {
  language: Language;
  setLanguage: (l: Language) => void;
  fontScale: number;
  setFontScale: (s: number) => void;
  highContrast: boolean;
  setHighContrast: (v: boolean) => void;
  t: typeof labels['vi'];
}) {
  return (
    <div className="settings-view">
      <h2 className="settings-title">{t.settings}</h2>

      <div className="setting-row">
        <div>
          <div className="setting-label">{t.language_label}</div>
          <div className="setting-desc">Vietnamese / English</div>
        </div>
        <button
          className="btn-secondary"
          onClick={() => setLanguage(language === 'vi' ? 'en' : 'vi')}
        >
          {language === 'vi' ? 'English' : 'Tieng Viet'}
        </button>
      </div>

      <div className="setting-row">
        <div>
          <div className="setting-label">{t.font_size}</div>
          <div className="setting-desc">{Math.round(fontScale * 100)}%</div>
        </div>
        <input
          type="range"
          className="font-scale-slider"
          min="0.8"
          max="1.5"
          step="0.1"
          value={fontScale}
          onChange={e => setFontScale(parseFloat(e.target.value))}
        />
      </div>

      <div className="setting-row">
        <div>
          <div className="setting-label">{t.high_contrast}</div>
        </div>
        <button
          className={'toggle-btn' + (highContrast ? ' active' : '')}
          onClick={() => setHighContrast(!highContrast)}
        ></button>
      </div>

      <div className="info-box">
        <h4>{t.about_robot}</h4>
        <p>
          {language === 'vi'
            ? 'Robot tu hanh su dung ROS 2, Mecanum, Nav2. Duoc phat trien boi sinh vien PTIT.'
            : 'Autonomous robot using ROS 2, Mecanum wheels, Nav2. Developed by PTIT students.'}
        </p>
      </div>

      <div className="info-box">
        <h4>{t.privacy_label}</h4>
        <p>
          {language === 'vi'
            ? 'Giao dien khach khong luu lich su cau hoi hoac du lieu ca nhan. Camera preview khong hien thi mac dinh.'
            : 'The visitor interface does not store question history or personal data. Camera preview is hidden by default.'}
        </p>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* FeedbackView                                                        */
/* ------------------------------------------------------------------ */
function FeedbackView({ setEmotion, goHome, t }: {
  setEmotion: (e: EmotionState) => void;
  goHome: () => void;
  t: typeof labels['vi'];
}) {
  const [rating, setRating] = useState<Rating>(null);
  const [category, setCategory] = useState<FeedbackCategory>(null);
  const [comment, setComment] = useState('');
  const [submitted, setSubmitted] = useState(false);

  const MAX_CHARS = 500;

  const handleSubmit = () => {
    // In Phase 6 this will POST to /api/v1/visitor/feedback
    console.log('Feedback:', { rating, category, comment });
    setSubmitted(true);
    setEmotion('arrived');
  };

  if (submitted) {
    return (
      <div className="thank-you">
        <EmotionFace state="arrived" />
        <h2>{t.thank_title}</h2>
        <p>{t.thank_sub}</p>
        <button className="btn-primary" onClick={goHome}>{t.home}</button>
      </div>
    );
  }

  const ratingOptions: { key: Rating; label: string }[] = [
    { key: 'great', label: t.rating_great },
    { key: 'good', label: t.rating_good },
    { key: 'ok', label: t.rating_ok },
    { key: 'bad', label: t.rating_bad },
  ];

  const categoryOptions: { key: FeedbackCategory; label: string }[] = [
    { key: 'nav', label: t.cat_nav },
    { key: 'info', label: t.cat_info },
    { key: 'ui', label: t.cat_ui },
    { key: 'other', label: t.cat_other },
  ];

  return (
    <div className="feedback-view">
      <h2 className="feedback-title">{t.feedback}</h2>

      <div>
        <div className="feedback-label">{t.rate_label}</div>
        <div className="rating-group">
          {ratingOptions.map(opt => (
            <button
              key={opt.key}
              className={'rating-btn' + (rating === opt.key ? ' selected' : '')}
              onClick={() => setRating(opt.key)}
            >
              {opt.label}
            </button>
          ))}
        </div>
      </div>

      <div>
        <div className="feedback-label">{t.category_label}</div>
        <div className="category-group">
          {categoryOptions.map(opt => (
            <button
              key={opt.key}
              className={'category-btn' + (category === opt.key ? ' selected' : '')}
              onClick={() => setCategory(opt.key)}
            >
              {opt.label}
            </button>
          ))}
        </div>
      </div>

      <div>
        <div className="feedback-label">{t.comment_label}</div>
        <textarea
          className="feedback-textarea"
          maxLength={MAX_CHARS}
          value={comment}
          onChange={e => setComment(e.target.value)}
        ></textarea>
        <div className="char-count">{comment.length}/{MAX_CHARS}</div>
      </div>

      <button
        className="btn-primary"
        disabled={!rating}
        onClick={handleSubmit}
      >
        {t.submit}
      </button>
    </div>
  );
}
