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

network DADCA_ACK_n100_u3_a8000m2_8
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
        @display("i=misc/sensor;p=822.17,7859.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1335.65,7990.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2214.87,7866.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3859.43,7906.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5372.95,7754.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=423.60,7557.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1564.17,7695.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2555.07,7704.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3534.80,7543.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4115.02,7616.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4777.33,7715.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1142.66,7318.58");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2163.65,7320.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2951.26,7342.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4566.81,7395.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5011.09,7350.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5450.05,7406.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5833.38,7327.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=32.26,7147.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=703.44,7011.92");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1809.35,7200.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2583.01,7113.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3324.47,7012.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4158.28,7067.75");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1529.81,6958.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1967.25,6833.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3752.82,6967.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4823.44,6983.41");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5218.39,6923.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5704.09,6963.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=211.98,6582.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=995.90,6603.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2439.55,6670.61");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3046.95,6618.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4062.66,6629.48");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4431.05,6694.47");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4887.64,6595.63");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1281.76,6353.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1692.11,6277.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2537.22,6329.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3512.30,6488.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5207.23,6424.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5658.57,6444.65");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=141.64,6125.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=717.89,6228.63");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2878.77,6243.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4449.79,6067.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4859.08,6134.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5917.73,6077.64");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=683.89,5806.30");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1339.45,5925.23");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1803.66,5870.43");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2317.96,5848.12");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2730.99,5882.56");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3397.75,6006.89");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3886.42,5948.30");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5398.66,5871.47");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=37.52,5768.59");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1524.38,5594.53");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2028.34,5577.65");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3051.03,5626.50");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4181.84,5622.77");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4759.68,5604.60");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5757.26,5755.95");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=334.85,5503.47");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=870.36,5457.42");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3688.92,5368.42");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5202.37,5519.88");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5472.03,5283.33");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1314.84,5118.67");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1798.90,5152.04");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2579.98,5077.16");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3042.84,5152.00");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3427.75,5070.56");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4378.93,5201.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5055.89,5125.88");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=167.47,4830.10");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=847.90,4931.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2188.98,4947.24");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4021.20,5010.97");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5296.64,4831.26");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5817.08,4948.42");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=516.60,4657.69");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=936.73,4536.92");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1436.70,4657.66");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1863.27,4633.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2214.31,4554.89");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2556.53,4690.17");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2915.08,4700.67");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3294.25,4730.90");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3979.44,4592.82");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4640.62,4648.80");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5524.09,4536.82");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1537.79,4294.09");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3149.60,4316.36");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3623.30,4501.26");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5112.57,4428.88");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=244.80,4178.00");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=951.96,4163.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2024.43,4217.71");
    }

}

