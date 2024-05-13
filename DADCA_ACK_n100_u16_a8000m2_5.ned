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

network DADCA_ACK_n100_u16_a8000m2_5
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
        @display("i=misc/sensor;p=902.62,7982.11");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2340.54,7933.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2919.05,7824.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4104.15,7843.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4908.22,7930.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5326.90,7764.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=337.06,7622.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=774.43,7625.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1292.27,7659.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1697.65,7735.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3207.95,7559.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3580.62,7524.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5786.22,7578.78");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=10.19,7354.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1030.81,7363.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1554.60,7304.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2050.35,7424.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2550.96,7393.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4522.77,7357.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5093.91,7478.43");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=456.68,7225.78");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2011.87,7034.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3329.01,7109.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4006.34,7225.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5511.53,7097.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=343.24,6861.01");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=949.00,6990.03");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2716.39,6812.28");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3841.23,6877.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4676.62,6940.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5044.52,6855.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=738.83,6535.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1361.37,6562.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1697.16,6676.73");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2250.21,6580.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3034.54,6602.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3506.87,6704.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4005.81,6536.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5767.25,6628.41");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=238.74,6414.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4608.43,6508.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5120.22,6497.06");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=921.33,6121.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1783.19,6120.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2349.52,6086.88");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3359.90,6200.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3881.82,6023.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4407.99,6143.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5199.13,6127.26");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1396.57,5954.93");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3001.43,5903.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5620.34,5941.07");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=328.55,5588.57");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1000.35,5575.75");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1364.44,5579.18");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1957.29,5768.62");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2398.99,5578.59");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2767.27,5631.40");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3558.93,5762.43");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4092.77,5674.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4747.49,5536.14");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5899.44,5545.30");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3868.99,5340.15");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4274.52,5323.28");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5140.91,5507.38");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5498.20,5300.15");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=93.19,5086.41");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=772.96,5050.70");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1657.01,5121.16");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2031.18,5214.74");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2831.70,5113.83");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3237.31,5251.51");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2018.64,4810.32");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2378.81,4824.08");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4489.38,5010.51");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4886.72,4918.92");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5578.60,4879.75");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=188.13,4605.81");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1606.21,4728.76");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3408.24,4715.88");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3849.08,4665.91");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4336.54,4646.68");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5292.71,4583.54");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5927.55,4684.33");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=525.46,4504.73");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1043.72,4474.46");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1586.20,4313.58");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2132.93,4435.17");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2876.29,4502.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3213.55,4384.91");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4670.14,4424.13");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=827.14,4187.50");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2605.65,4194.39");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3649.77,4172.70");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4210.94,4238.50");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5079.35,4099.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5718.55,4273.34");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=341.16,3853.80");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1398.10,3830.80");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2038.18,3855.18");
    }

}

