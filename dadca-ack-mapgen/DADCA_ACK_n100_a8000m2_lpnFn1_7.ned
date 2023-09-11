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
        @display("i=misc/sensor,p=99.37,180.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=26.62,164.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=13.71,79.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=46.27,255.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=166.09,260.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=222.53,229.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=219.76,155.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=219.82,327.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=309.68,362.39");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=346.28,449.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=381.55,356.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=478.37,334.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=431.03,402.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=583.79,400.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=524.80,462.37");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=618.93,493.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=658.10,421.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=708.42,458.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=633.01,348.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=592.36,299.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=612.63,228.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=569.39,180.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=551.42,45.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=474.08,70.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=435.64,172.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=492.59,192.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=371.89,92.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=316.70,156.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=336.50,238.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=373.31,177.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=273.99,282.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=135.81,382.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=107.67,282.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=410.58,292.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=210.22,441.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=376.71,507.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=461.53,490.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=402.76,563.03");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=448.20,609.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=504.55,588.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=563.55,607.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=572.07,542.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=781.05,381.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=831.86,336.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=920.03,320.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1000.44,323.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1035.30,394.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1057.62,297.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1134.46,299.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1055.96,204.85");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1126.43,208.13");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1105.84,111.97");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=962.91,438.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1027.06,491.03");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=878.69,480.81");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=786.57,488.85");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=546.36,346.28");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=383.08,620.58");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=409.52,713.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=272.90,694.27");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=281.35,617.10");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=230.49,582.30");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=343.80,788.16");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=281.04,779.56");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=248.47,839.06");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=181.91,810.80");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=161.14,737.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=184.55,643.15");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=223.39,760.49");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=187.55,890.87");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=112.94,889.25");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=130.71,975.73");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=198.37,1045.49");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=110.88,1044.82");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=124.66,1146.41");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=184.72,1117.12");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=171.74,1189.86");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=235.06,1177.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=27.98,1050.60");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=61.62,969.48");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=44.02,1189.23");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=197.57,982.70");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=265.17,1005.43");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=300.51,938.27");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=63.94,1129.07");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=247.08,1094.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=151.99,1249.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=372.80,862.45");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=254.12,490.08");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=85.59,423.67");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=139.40,474.29");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=81.37,528.42");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=85.49,625.50");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=18.39,589.41");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=9.94,684.39");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=643.28,630.59");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=611.04,687.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=620.55,759.93");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=539.74,726.30");
    }

}

