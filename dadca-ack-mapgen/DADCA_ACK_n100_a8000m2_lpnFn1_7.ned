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
        @display("i=misc/sensor,p=152.79,38.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=11.85,96.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=76.30,40.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=19.07,201.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=87.07,252.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=23.41,273.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=195.92,99.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=261.79,47.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=144.45,151.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=148.79,347.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=82.62,327.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=54.51,384.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=278.25,132.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=342.86,134.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=339.59,226.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=291.65,270.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=358.20,328.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=413.75,396.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=474.85,329.77");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=544.78,294.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=595.48,337.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=582.27,418.64");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=521.46,438.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=469.27,208.07");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=408.19,188.44");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=242.55,207.31");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=184.95,261.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=232.17,356.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=165.01,423.62");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=534.58,200.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=481.80,147.99");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=574.53,119.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=591.60,21.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=680.72,59.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=711.92,122.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=798.03,88.54");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=861.75,73.84");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=812.31,1.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=904.52,16.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=966.39,70.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=941.92,150.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=954.33,212.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=875.42,213.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=875.29,142.63");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1009.97,188.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1056.16,264.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=995.34,277.97");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=987.97,340.18");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=915.21,282.20");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=932.66,366.60");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=871.31,345.10");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=815.14,461.60");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=757.10,414.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=870.76,484.21");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=800.49,538.73");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=732.39,541.98");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=757.61,630.11");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=776.18,708.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=725.17,779.40");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=770.68,843.87");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=688.55,831.49");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=737.00,900.93");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=694.82,961.33");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=635.84,949.82");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=540.99,924.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=616.95,832.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=625.97,764.20");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=870.33,551.08");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=847.70,619.06");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=850.66,714.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=815.99,914.50");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=815.18,1036.85");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=870.03,1110.30");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=946.96,918.56");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1013.65,853.07");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1070.72,749.75");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1151.27,715.37");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1118.73,664.85");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1125.00,605.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1204.84,636.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1283.60,626.83");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1263.27,552.20");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1347.52,498.94");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1300.35,457.95");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1359.07,378.58");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1448.11,335.62");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1512.09,390.15");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1545.06,307.31");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1590.83,359.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1580.06,472.88");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1647.08,538.50");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1698.05,502.40");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1681.86,592.67");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1486.73,494.09");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1447.67,427.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1392.01,549.88");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1462.66,600.35");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1485.65,668.78");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1568.21,723.43");
    }

}

