import {html, LitElement, nothing} from 'lit'
import {customElement, state} from 'lit/decorators.js'
import '../assets/mui.js'
import {postWithFeedback, saveJSONWithReboot} from '../utils/feedback.js'
import '../components/filedrag.js'
import IMU_SCHEMA from '../utils/imu-schema.js'
import {_arrayInput, _intInput, _uintInput} from "../utils/libs.js";

@customElement('imu-panel')
export class IMUPanel extends LitElement {

    @state() accessor imu_customised = false

    static SCHEMA = IMU_SCHEMA

    createRenderRoot() {
        return this
    }

    render() {
        return html`
            <div class="imu-layout">
                <div class="mui-panel mui--text-title">IMU (Gyro)</div>
                <div class="mui-panel">
                    Upload target configuration (remember to press "Save IMU Configuration" at the bottom of the page):
                    <p>
                    <file-drop id="filedrag" label="Upload" @file-drop=${this._onFileDrop}>or drop files here</file-drop>
                </div>
                <div class="mui-panel">
                    <div class="mui-panel"
                         style="display:${this.imu_customised ? 'block' : 'none'}; background-color: #FFC107;">
                        This IMU configuration has been customized. This can be safely ignored if this is a custom
                        build or for testing purposes.<br>
                        You can <a download href="/imu.json">download</a> the configuration or
                        <a href="/reset?imu" @click="${postWithFeedback('IMU Configuration Reset', 'Reset failed', '/reset?imu')}">reset</a>
                        to pre-configured IMU defaults and reboot.
                    </div>
                    <form id="upload_imu" class="mui-form">
                        ${this._renderTable()}
                        <br>
                        <input type="button" name="_ignore" value="Save IMU  Configuration"
                               class="mui-btn mui-btn--primary" @click=${this._submitConfig} />
                    </form>
                </div>
            </div>
        `
    }

    _renderBadge(badgeType) {
        const BADGES = {
            'missing-type': {
                'text': 'MISSING',
                'bg': 'red',
                'fg': 'white'
            }
        } 
        return html`
            <span
                class="badge"
                id="${badgeType}"
                style="background-color: ${BADGES[badgeType]['bg']}; color: ${BADGES[badgeType]['fg']}"
            >
                ${BADGES[badgeType]['text']}
            </span>
        `
    }

    _renderTable() {
        return html`
            <table>
                <tbody>
                ${this.constructor.SCHEMA.map(group => html`
                    <tr>
                        <td colspan="4" class="mui--text-title">${group.title}</td>
                    </tr>
                    ${group.sections.map(section => html`
                        <tr>
                            <td colspan="4"><b>${section.title}</b></td>
                        </tr>
                        ${section.rows.map(row => html`
                            <tr>
                                <td width="30"></td>
                                <td>${row.label}${this._renderIcon(row.icon)}</td>
                                <td>${this._renderField(row)}</td>
                                <td>${row.desc || ''}</td>
                            </tr>
                        `)}
                    `)}
                    <tr><br></tr>
                `)}
                </tbody>
            </table>
        `
    }

    _renderIcon(icon) {
        if (!icon) return html``
        if (icon === 'input-output') {
            return html`<img class="icon-input"/><img class="icon-output"/>`
        }
        return html`<img class="icon-${icon}"/>`
    }

    _renderField(row) {
        switch (row.type) {
            case 'checkbox':
                return html`<input id="${row.id}" name="${row.id}" type="checkbox"/>`
            case 'select':
                return html`<select id="${row.id}" name="${row.id}">
                    ${row.options?.map(opt => html`
                        <option value="${opt.value}">${opt.label}</option>`)}
                </select>`
            case 'read-only-select':
                return html`<select id="${row.id}" name="${row.id}" disabled>
                    ${row.options?.map(opt => html`
                        <option value="${opt.value}">${opt.label}</option>`)}
                </select>`
            case 'int':
                return html`<input id="${row.id}" name="${row.id}" size=${row.size ?? 3} maxlength=${row.size ?? 3} type="text" @keypress="${_intInput}"/>`
            case 'uint':
                if (!row.unit) return html`<input id="${row.id}" name="${row.id}" size=${row.size ?? 3} maxlength=${row.size ?? 3} type="text" @keypress="${_uintInput}"/>`
                return html`<input id="${row.id}" name="${row.id}" size=${row.size ?? 3} maxlength=${row.size ?? 3} type="text" @keypress="${_uintInput}"/>&nbsp;<span>${row.unit}</span>`
            case 'array':
                return html`<input id="${row.id}" name="${row.id}" size=${row.size ?? nothing} maxlength=${row.size ?? nothing} type="text" class="array"  @keypress="${_arrayInput}"/>`
            case 'presence':
                return html`
                _renderBadge(${_intInput})`
            case 'spacer':
                return html`
                <br>
                `
        }
    }

    connectedCallback() {
        super.connectedCallback()
        // Add tooltips to icon classes after first paint
        setTimeout(() => this._initTooltips(), 0)
        this._loadData()
    }

    _initTooltips() {
        const add = (cls, label) => {
            const images = document.querySelectorAll('.' + cls)
            images.forEach(i => i.setAttribute('title', label))
        }
        add('icon-input', 'Digital Input')
        add('icon-output', 'Digital Output')
        add('icon-analog', 'Analog Input')
        add('icon-pwm', 'PWM Output')
    }

    _loadData() {
        const xmlhttp = new XMLHttpRequest()
        xmlhttp.onreadystatechange = () => {
            if (xmlhttp.readyState === 4 && xmlhttp.status === 200) {
                const data = JSON.parse(xmlhttp.responseText)
                this.imu_customised = !!data.imu_customised
                this._updateIMUSettings(data)
            }
        }
        xmlhttp.open('GET', '/imu.json', true)
        xmlhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded')
        xmlhttp.send()
    }

    _onFileDrop(e) {
        const files = e.detail.files
        const form = document.getElementById('upload_imu')
        if (form) form.reset()
        for (const file of files) {
            const reader = new FileReader()
            reader.onload = (ev) => {
                const data = JSON.parse(ev.target.result)
                this._updateIMUSettings(data)
            }
            reader.readAsText(file)
        }
    }

    _updateIMUSettings(data) {
        for (const [key, value] of Object.entries(data)) {
            const el = document.getElementById(key)
            if (el) {
                if (el.type === 'checkbox') {
                    el.checked = !!value
                } else {
                    if (Array.isArray(value)) el.value = value.toString()
                    else el.value = value
                }
            }
        }
    }

    _submitConfig() {
        const form = document.getElementById('upload_imu')
        const formData = new FormData(form)
        // rebuild using original serializer logic
        const body = JSON.stringify(Object.fromEntries(formData), (k, v) => {
            if (v === '') return undefined
            const el = document.getElementById(k)
            if (el && el.type === 'checkbox') {
                return v === 'on'
            }
            if (el && el.classList.contains('array')) {
                const arr = v.split(',').map((element) => Number(element))
                return arr.length === 0 ? undefined : arr
            }
            return isNaN(v) ? v : +v
        })
        // Use shared helper that prompts for reboot on success
        saveJSONWithReboot('Upload Succeeded', 'Upload Failed', '/imu.json', {...JSON.parse(body), "imu_customised": true})
        return false
    }
}
