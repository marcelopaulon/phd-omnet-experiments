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

network DADCA_ACK_n100_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=104.60,0.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=1.87,16.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=216.37,123.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=154.44,150.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=93.21,185.96");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=31.70,83.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=163.57,19.58");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=276.40,27.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=360.53,71.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=410.70,23.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=482.49,90.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=421.34,148.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=324.81,165.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=297.78,93.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=242.64,193.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=296.79,227.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=362.18,239.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=287.12,373.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=207.44,381.90");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=195.56,321.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=221.97,250.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=321.16,429.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=266.22,307.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=131.92,252.61");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=44.89,249.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=118.42,337.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=41.83,333.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=49.95,412.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=139.66,425.56");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=339.09,297.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=401.92,348.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=64.63,523.07");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=80.24,608.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=20.82,583.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=8.23,470.45");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=154.56,593.51");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=139.22,684.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=97.52,743.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=9.90,791.29");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=84.87,811.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=22.20,854.84");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=20.60,661.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1.61,723.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=129.92,510.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=202.49,498.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=267.56,487.58");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=336.69,559.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=382.64,625.24");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=417.06,558.74");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=449.80,717.93");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=517.32,742.25");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=522.28,858.16");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=599.96,714.73");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=539.59,651.15");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=455.10,655.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=658.30,749.82");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=582.88,795.32");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=412.20,818.56");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=347.23,706.22");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=278.15,619.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=213.71,718.08");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=275.55,686.11");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=204.10,639.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=220.68,814.78");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=137.10,863.53");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=351.79,831.52");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=324.22,772.51");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=268.56,919.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=291.12,854.38");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=361.08,906.65");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=316.35,994.80");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=412.43,993.27");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=165.30,789.87");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=146.97,959.57");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=51.15,975.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=11.55,1032.57");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=156.29,1059.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=249.80,1024.78");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=204.46,1163.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=134.56,1195.35");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=332.89,1058.27");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=86.65,913.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=501.30,555.56");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=585.81,559.80");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=432.77,462.01");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=644.86,611.80");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=729.61,592.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=735.63,667.04");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=755.17,517.01");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=695.75,438.72");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=704.04,362.86");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=783.26,358.43");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=866.77,353.35");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=953.85,330.21");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=803.24,435.92");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=913.83,423.78");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1004.14,418.04");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1086.30,394.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1053.11,496.60");
    }

}

