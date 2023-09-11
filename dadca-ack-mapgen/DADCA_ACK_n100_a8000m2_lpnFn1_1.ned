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
        @display("i=misc/sensor,p=91.59,172.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=157.47,189.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=160.06,83.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=180.05,26.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=118.02,3.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=245.91,16.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=323.51,67.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=336.34,166.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=24.24,225.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=66.91,276.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=16.07,331.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=213.18,145.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=225.13,206.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=397.41,113.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=413.72,53.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=488.10,59.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=18.95,17.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=17.45,99.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=298.33,269.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=355.91,304.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=394.39,385.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=433.43,217.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=517.08,207.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=556.61,279.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=480.45,316.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=554.49,87.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=614.48,124.14");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=674.86,111.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=686.38,34.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=631.49,208.81");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=774.94,74.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=796.59,12.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=777.98,162.64");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=876.09,172.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=878.08,235.40");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=900.76,335.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=837.16,341.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=959.05,229.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=929.23,140.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=995.96,160.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1000.35,313.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1020.76,241.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1067.80,204.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1054.74,116.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=972.31,81.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=708.45,242.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=691.34,310.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=805.94,273.23");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=565.89,166.26");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=453.00,139.79");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=430.91,279.21");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=270.47,326.13");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=152.25,252.87");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=173.88,343.89");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=106.23,390.11");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=231.82,274.29");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=112.58,318.34");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=132.94,479.40");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=101.01,551.76");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=107.62,626.00");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=180.46,602.35");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=265.93,574.30");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=271.83,646.06");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=365.00,654.17");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=216.17,725.44");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=284.41,760.57");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=416.90,715.52");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=443.63,627.72");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=190.28,670.46");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=124.70,719.67");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=78.74,766.02");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=50.15,427.77");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=40.26,608.56");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=33.57,529.99");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=203.83,511.05");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=261.22,461.69");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=315.98,370.95");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=237.15,406.28");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=23.69,697.96");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=362.52,454.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=480.15,402.99");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=301.22,518.15");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=229.53,86.75");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=278.21,124.23");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=596.25,8.93");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=495.83,469.06");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=582.92,357.40");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=581.39,454.62");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=573.52,515.08");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=673.46,483.16");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=633.28,553.02");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=555.25,582.38");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=557.76,642.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=156.76,776.40");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=164.06,411.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=636.08,640.84");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=484.03,559.60");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=408.53,507.29");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=381.39,776.68");
    }

}

