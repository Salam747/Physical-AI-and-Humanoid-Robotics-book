import React from 'react';
import {useThemeConfig} from '@docusaurus/theme-common';
import {
  splitNavbarItems,
  useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarItem from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import SearchBar from '@theme/SearchBar';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarSearch from '@theme/Navbar/Search';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './styles.module.css';

function useNavbarItems() {
  return useThemeConfig().navbar.items;
}

function NavbarItems({items}) {
  return (
    <>
      {items.map((item, i) => (
        <NavbarItem {...item} key={i} />
      ))}
    </>
  );
}

function NavbarContentLayout({left, right}) {
  return (
    <div className="navbar__inner">
      <div className="navbar__items">{left}</div>
      <div className="navbar__items navbar__items--right">{right}</div>
    </div>
  );
}

export default function NavbarContent() {
  const mobileSidebar = useNavbarMobileSidebar();
  const items = useNavbarItems();
  const [leftItems, rightItems] = splitNavbarItems(items);
  const { user, isAuthenticated, logout, loading } = useAuth();

  const searchBarItem = items.find((item) => item.type === 'search');

  // Filter out Login link if user is authenticated
  const filteredRightItems = isAuthenticated
    ? rightItems.filter(item => item.label !== 'Login')
    : rightItems;

  return (
    <NavbarContentLayout
      left={
        <>
          <NavbarLogo />
          <NavbarItems items={leftItems} />
        </>
      }
      right={
        <>
          {/* Show loading skeleton or actual items */}
          {loading ? (
            <div className={styles.loadingSkeleton}>
              <div></div>
            </div>
          ) : (
            <NavbarItems items={filteredRightItems} />
          )}

          {!searchBarItem && (
            <NavbarSearch>
              <SearchBar />
            </NavbarSearch>
          )}

          {/* Custom Auth Section - Show immediately when authenticated */}
          {!loading && isAuthenticated && user && (
            <div className={styles.authSection}>
              <button
                onClick={logout}
                className={styles.logoutButton}
                title="Logout"
              >
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M9 21H5C4.46957 21 3.96086 20.7893 3.58579 20.4142C3.21071 20.0391 3 19.5304 3 19V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H9" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M16 17L21 12L16 7" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M21 12H9" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
                <span>Logout</span>
              </button>
              <div className={styles.userInfo}>
                <span className={styles.userIcon}>👤</span>
                <span className={styles.userName}>{user.first_name}</span>
                <span className={styles.statusDot}></span>
              </div>
            </div>
          )}

          {/* Always show color mode toggle */}
          <NavbarColorModeToggle className="navbar__item" />
        </>
      }
    />
  );
}
