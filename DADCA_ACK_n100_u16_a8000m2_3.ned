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

network DADCA_ACK_n100_u16_a8000m2_3
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
        @display("i=misc/sensor;p=126.31,7908.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1269.85,7763.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1711.70,7952.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2407.11,7922.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3682.24,7893.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=654.08,7570.34");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1594.99,7596.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2088.62,7686.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2927.30,7523.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3427.63,7548.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3908.76,7624.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4262.48,7744.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=232.39,7365.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4907.20,7476.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1072.74,7257.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1444.31,7239.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2474.53,7170.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3595.36,7151.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3945.29,7179.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4349.63,7192.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5177.34,7166.38");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=369.76,6873.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=868.45,6854.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1271.56,6852.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2007.90,6960.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3126.29,6925.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3856.28,6767.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2322.38,6715.77");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2673.89,6611.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3388.92,6682.06");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4757.84,6721.94");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5135.47,6611.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5808.98,6730.38");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=131.99,6431.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=903.12,6487.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1376.19,6436.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1726.28,6376.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2256.62,6344.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3979.14,6424.13");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4383.53,6503.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5477.51,6399.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=456.49,6229.20");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1214.27,6097.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3065.69,6164.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3508.99,6232.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3888.81,6051.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4322.99,6152.58");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4991.77,6222.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5345.07,6024.26");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5748.05,6167.58");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=285.90,5838.58");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1625.55,5938.39");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1992.28,5872.83");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2730.57,6014.79");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4265.11,5799.18");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=742.40,5687.30");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1232.99,5711.86");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1752.38,5591.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2496.94,5726.22");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3034.04,5680.70");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3465.02,5751.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4787.94,5766.64");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5099.65,5546.38");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5658.63,5591.99");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1074.56,5324.19");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2704.07,5357.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3316.10,5293.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3795.48,5495.04");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4449.82,5471.33");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=417.58,5167.99");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1749.75,5234.49");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2304.15,5094.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2917.00,5054.72");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4096.36,5030.85");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5382.60,5080.24");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5944.95,5179.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=118.32,4848.61");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3230.60,4819.40");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4917.57,4897.93");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1250.66,4606.12");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1799.83,4659.17");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2570.10,4613.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5303.14,4679.16");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5878.25,4557.20");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=467.16,4505.93");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3741.38,4453.99");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4281.36,4299.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4810.49,4415.86");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5267.65,4292.45");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=727.13,4266.90");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1558.20,4134.13");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2130.80,4161.94");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3066.58,4247.34");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3667.13,4100.81");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5751.15,4045.50");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1229.30,3815.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2577.34,3942.29");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4024.36,3837.68");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4788.06,4015.43");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=222.56,3772.38");
    }

}

