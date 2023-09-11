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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor;p=7.52,112.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=7.19,36.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=70.64,15.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=168.37,132.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=189.88,63.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=213.87,190.69");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=274.14,139.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=340.23,115.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=397.02,85.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=457.13,82.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=452.78,11.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=528.09,133.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=288.05,201.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=230.62,300.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=304.96,347.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=369.14,353.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=400.46,425.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=481.23,434.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=445.50,501.06");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=374.03,533.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=514.98,500.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=594.83,364.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=599.93,268.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=447.55,357.96");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=324.82,572.27");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=363.79,633.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=425.04,707.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=473.68,620.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=537.40,694.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=614.29,703.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=674.46,637.66");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=567.25,607.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=600.17,552.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=777.22,616.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=768.98,706.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=828.92,774.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=742.87,781.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=701.88,847.40");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=642.39,869.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=615.73,797.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=556.33,864.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=481.98,926.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=557.18,929.59");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=645.98,946.60");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=664.72,1013.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=720.89,982.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=772.65,847.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=825.57,668.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=693.93,505.28");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=634.25,449.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=714.40,569.76");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=794.26,554.75");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=892.63,561.88");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=475.22,766.41");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=781.56,479.47");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=699.01,383.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=702.30,320.78");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=765.66,382.31");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=778.52,293.74");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=798.92,222.58");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=848.05,274.61");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=945.26,278.58");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=920.67,338.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1006.87,299.39");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=969.68,374.52");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=908.92,408.71");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=911.07,218.33");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=989.81,204.34");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=988.48,123.72");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1102.63,248.97");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1111.91,118.12");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1160.49,71.88");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1225.40,89.10");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1185.35,13.46");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1178.10,140.05");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1108.46,41.70");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1020.11,56.53");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1295.69,27.95");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1284.22,114.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1305.16,177.69");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1337.03,251.99");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1280.86,273.91");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1247.16,347.09");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1222.30,414.01");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1194.99,495.56");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1278.47,484.84");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1312.02,432.65");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1183.26,302.60");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1161.93,205.25");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1245.87,222.01");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=912.13,147.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1351.48,491.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1401.07,535.07");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1462.17,543.15");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1492.93,475.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1416.23,439.32");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1462.31,388.78");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1550.27,420.31");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1583.06,353.79");
    }

}

