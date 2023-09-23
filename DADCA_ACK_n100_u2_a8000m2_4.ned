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

network DADCA_ACK_n100_u2_a8000m2_4
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
        @display("i=misc/sensor;p=204.45,7869.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1566.69,7994.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2559.66,7822.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3330.00,7952.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3863.77,7907.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4410.66,7871.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5102.17,7889.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=539.61,7566.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=894.27,7618.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1249.61,7743.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2007.07,7703.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2816.09,7543.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=117.74,7485.78");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1673.64,7352.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2272.74,7435.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3478.51,7470.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3896.21,7274.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4238.96,7465.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4835.53,7345.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5462.70,7499.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=418.31,7228.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=999.75,7146.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1444.37,7031.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1979.49,7104.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3315.15,7088.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4128.02,7010.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=458.67,6857.99");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2659.59,7001.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3004.51,6876.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4691.29,6791.06");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5171.80,6889.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5596.35,6952.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=96.07,6601.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=784.45,6515.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1859.69,6728.20");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3692.31,6579.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4040.50,6640.40");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5543.18,6519.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5950.81,6607.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1446.38,6415.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1973.72,6378.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2566.62,6353.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3139.40,6481.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=309.34,6150.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1112.45,6060.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4284.65,6257.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4663.45,6248.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5033.24,6234.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5580.13,6049.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1692.15,5812.39");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2344.85,5795.33");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2963.39,5983.66");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3610.08,6017.54");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4292.12,5893.32");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5950.54,5972.75");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3061.14,5601.86");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4893.93,5721.96");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5322.80,5721.15");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=26.43,5342.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=539.31,5336.42");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1121.64,5430.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1760.50,5317.10");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2501.43,5411.96");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3591.67,5386.06");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4447.75,5312.47");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5893.53,5524.04");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=841.47,5056.76");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1401.25,5132.92");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4968.00,5246.25");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5406.65,5071.02");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5773.19,5060.08");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=383.36,4854.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2262.23,4934.46");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2774.88,4981.77");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3840.27,4971.09");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4585.87,4980.79");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5127.60,4831.00");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=899.94,4624.03");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1491.50,4769.30");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2011.40,4573.23");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3203.52,4764.86");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3869.56,4605.46");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4254.95,4676.51");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4838.65,4595.45");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=0.55,4313.09");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1232.57,4509.25");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2486.33,4301.53");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2799.71,4485.17");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5386.64,4386.50");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5785.23,4342.80");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=657.37,4160.14");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1502.54,4203.11");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3224.45,4040.85");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3772.33,4264.23");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4182.84,4145.47");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=147.59,3978.51");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1376.30,3862.06");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1794.54,3883.97");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2379.74,3902.97");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2893.64,3856.28");
    }

}

