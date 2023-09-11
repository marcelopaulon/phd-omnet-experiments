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

network DADCA_ACK_n100_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=110.01,34.44");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=179.64,22.05");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=255.70,3.15");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=239.23,68.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=307.58,134.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=353.36,179.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=318.62,237.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=285.29,308.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=198.40,340.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=212.35,232.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=225.84,173.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=352.38,79.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=402.00,24.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=466.95,16.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=466.15,78.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=571.16,62.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=643.01,53.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=609.88,114.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=591.01,179.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=553.00,232.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=497.72,201.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=442.37,330.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=521.13,368.68");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=356.48,374.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=309.89,462.07");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=220.38,434.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=246.30,500.35");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=157.46,465.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=147.10,400.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=70.26,316.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=49.69,375.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=133.00,279.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=128.76,208.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=49.64,230.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=49.80,152.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=162.19,106.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=430.71,185.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=369.28,306.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=475.13,411.59");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=407.80,476.72");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=407.61,568.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=471.42,504.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=526.98,635.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=520.12,733.59");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=593.05,730.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=612.61,650.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=588.36,472.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=543.26,526.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=598.71,582.03");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=643.51,533.34");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=717.03,496.26");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=781.92,555.59");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=777.52,624.00");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=799.01,488.26");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=821.10,416.63");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=784.32,329.23");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=873.77,310.61");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=929.92,253.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1006.01,305.52");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1020.30,401.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=935.47,394.44");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=910.59,455.77");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=898.20,519.05");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=825.45,190.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=758.76,182.96");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=660.92,201.52");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=736.53,120.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=826.57,115.20");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=846.48,46.86");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=682.98,3.35");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=634.73,291.19");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=765.52,10.26");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=932.59,150.79");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=935.57,60.19");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1001.10,8.89");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1070.70,3.70");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=992.38,122.32");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1054.07,185.24");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1125.51,129.50");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1201.91,135.45");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1146.77,64.24");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1059.89,68.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1086.27,237.29");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1114.67,309.79");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1183.21,276.71");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1245.87,314.89");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1161.04,354.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1111.30,391.37");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1068.84,471.88");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=975.82,475.01");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1032.47,528.62");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=988.49,601.22");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=903.72,635.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=955.42,537.97");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=855.42,676.59");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=794.26,693.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=881.09,733.26");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=984.97,723.89");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=738.33,743.34");
    }

}

