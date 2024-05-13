package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u4_a8000m2_1
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=1358.09,7800.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2048.36,7816.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3242.81,7957.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3530.32,7752.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4186.09,7786.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4545.26,7999.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4842.60,7778.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5335.49,7840.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=553.48,7696.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=935.45,7657.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1701.82,7616.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2658.14,7729.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5456.18,7505.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5956.72,7718.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=106.24,7447.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3244.09,7367.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4233.48,7426.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4552.85,7273.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5953.56,7262.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=696.44,7109.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1273.97,7198.90");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1989.49,7233.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2487.42,7091.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3764.15,7139.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4961.37,7234.19");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1659.38,7005.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2026.30,6764.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3419.61,6853.96");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4381.08,6921.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5457.91,6900.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=220.03,6751.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=998.94,6673.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1537.76,6598.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3839.61,6523.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5692.96,6636.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=24.40,6395.20");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=605.51,6466.14");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2309.24,6476.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2894.14,6437.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4323.48,6508.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4889.79,6493.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5306.27,6352.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=345.60,6224.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1312.33,6166.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1999.33,6261.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2744.25,6023.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3534.99,6198.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4024.49,6062.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4690.19,6200.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=45.83,5828.18");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1077.00,5860.62");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1945.27,5913.43");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3038.54,5831.71");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4408.97,5987.51");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5269.96,5857.80");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5643.05,6017.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=588.06,5667.00");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1457.08,5716.31");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2397.47,5698.95");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3598.68,5761.88");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3922.64,5618.61");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4764.28,5573.69");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=228.78,5469.76");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1157.81,5298.93");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1782.71,5346.71");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3382.66,5387.38");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4269.64,5501.71");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5471.09,5359.75");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5845.22,5518.61");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=187.04,5058.80");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=572.13,5108.78");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2525.91,5151.62");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2908.30,5217.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3824.78,5223.41");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4459.58,5173.23");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5131.17,5205.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1135.60,4832.92");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1841.47,4796.01");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2149.45,5010.87");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2603.44,4790.21");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3329.93,5012.07");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4100.84,4832.07");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4638.55,4806.50");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5019.93,4851.54");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5565.86,4957.30");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=439.09,4638.69");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3148.73,4603.21");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3686.33,4583.86");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1266.58,4355.68");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1801.30,4299.57");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2137.62,4404.21");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4794.44,4462.65");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5167.86,4476.54");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=322.59,4123.46");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=720.57,4158.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2950.95,4133.62");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3307.57,4180.71");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4201.76,4094.05");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5614.32,4245.87");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1581.77,3882.99");
    }

}

