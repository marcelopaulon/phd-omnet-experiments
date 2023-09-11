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

network DADCA_ACK_n100_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=148.48,22.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=10.52,55.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=78.82,23.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=143.92,145.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=249.55,51.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=242.51,131.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=338.86,50.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=421.11,42.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=497.63,67.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=553.05,97.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=459.72,119.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=535.25,3.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=424.26,193.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=561.42,180.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=328.59,139.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=374.50,228.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=473.55,7.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=400.01,101.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=594.30,297.11");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=650.27,259.22");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=691.81,337.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=725.56,271.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=701.04,203.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=533.30,246.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=528.05,344.58");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=473.79,388.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=616.52,435.19");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=675.36,420.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=857.66,271.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=791.83,212.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=848.30,151.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=801.42,97.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=836.23,8.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=738.12,17.93");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=631.74,161.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=441.51,304.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=358.65,289.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=698.90,98.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=641.17,18.92");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=294.38,197.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=208.87,192.51");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=169.20,278.84");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=132.15,214.75");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=65.57,226.76");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=52.45,307.31");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=225.73,333.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=175.32,394.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=249.25,394.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=225.23,473.59");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=86.18,387.30");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=26.93,445.72");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=5.90,377.37");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=296.53,489.50");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=373.37,430.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=458.40,479.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=528.31,499.67");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=537.70,438.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=480.28,562.33");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=568.65,609.38");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=408.96,519.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=362.00,601.66");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=425.23,609.15");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=299.82,571.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=222.80,606.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=300.50,645.81");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=520.83,689.66");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=431.60,731.29");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=642.59,555.70");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=690.72,501.65");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=748.75,434.84");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=813.65,446.99");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=283.95,265.81");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=456.01,245.16");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=349.48,369.35");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=293.46,325.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=182.81,89.39");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=25.18,164.26");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=574.91,388.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=142.91,335.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=11.03,544.24");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=6.52,629.36");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=58.50,671.36");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=95.30,732.10");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=177.63,737.34");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=203.67,679.50");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=101.41,583.41");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=152.09,482.31");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=412.88,357.87");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=938.35,259.36");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=964.87,315.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1026.85,390.91");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1032.26,314.76");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1041.91,240.69");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=965.47,160.21");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=905.18,124.98");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=936.82,27.28");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=987.69,69.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1050.52,25.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=787.64,316.16");
    }

}

