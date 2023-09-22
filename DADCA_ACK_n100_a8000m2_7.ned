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

network DADCA_ACK_n100_a8000m2_7
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
        @display("i=misc/sensor;p=1997.51,7881.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4124.24,7936.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=805.24,7723.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1335.36,7614.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1682.81,7712.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2578.55,7733.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4388.69,7509.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4803.94,7743.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5982.08,7724.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=472.37,7391.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1031.43,7392.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1615.98,7340.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1979.40,7432.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3077.94,7504.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5105.21,7440.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5671.26,7430.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2552.82,7242.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3294.88,7101.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3989.99,7049.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4937.67,7129.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5998.11,7137.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=370.41,6802.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=712.18,6936.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1096.23,6916.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1601.85,6848.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4491.48,6996.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5555.82,6784.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2204.81,6738.48");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2578.72,6553.22");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3007.30,6719.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3833.97,6735.29");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4962.20,6557.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5900.84,6534.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=615.29,6356.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1160.57,6476.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1533.28,6412.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2199.55,6384.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3459.98,6384.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3929.31,6336.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4377.73,6474.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5299.45,6380.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2340.18,6061.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4540.81,6089.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5020.63,6155.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5586.80,6177.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=234.55,5911.42");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=719.93,5990.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1333.37,5949.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1933.27,5974.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3038.49,5882.41");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3476.80,5916.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4198.90,5834.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4886.20,5829.39");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2454.72,5726.73");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3413.58,5560.28");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4368.48,5527.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5243.64,5688.91");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5599.15,5764.81");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=182.82,5367.55");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1212.22,5433.10");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1582.87,5499.06");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1980.19,5524.28");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2844.79,5424.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3809.19,5451.85");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4822.00,5293.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5383.18,5330.80");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5796.97,5287.47");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=535.47,5172.57");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1762.11,5098.85");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3208.93,5115.29");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3887.88,5052.79");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=877.19,4992.70");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2239.21,4821.97");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4256.73,4994.20");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4762.09,4867.42");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5199.21,5001.94");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5494.60,4795.41");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=193.07,4766.66");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2600.38,4777.20");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3212.36,4658.98");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3638.30,4550.20");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5247.67,4546.15");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=672.12,4466.66");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1088.83,4530.33");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1474.72,4477.29");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3358.31,4331.66");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4568.11,4402.05");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=306.50,4280.34");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1459.59,4048.41");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1938.32,4250.49");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2745.72,4180.70");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4142.50,4228.74");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5498.58,4146.47");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5861.57,4062.84");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2982.44,3882.19");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3507.61,3989.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3854.87,3913.21");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4556.84,4000.85");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5121.21,3971.00");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=288.95,3648.27");
    }

}

