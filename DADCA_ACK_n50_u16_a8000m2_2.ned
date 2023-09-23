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

network DADCA_ACK_n50_u16_a8000m2_2
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
        @display("i=misc/sensor;p=48.09,7782.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=549.37,7816.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=977.33,7991.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1347.10,7954.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1768.70,7791.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2963.43,7831.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4447.57,7993.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4839.16,7913.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3346.79,7702.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3812.23,7732.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4190.03,7600.44");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5604.64,7591.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=181.33,7454.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1108.89,7492.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1901.63,7276.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2243.71,7504.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2616.66,7321.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5027.35,7406.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5963.87,7320.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=258.49,7085.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=734.44,7173.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1281.26,7107.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3176.20,7208.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4206.72,7169.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5415.22,7036.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1967.77,6897.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2309.15,6769.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2662.12,6849.14");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3573.87,6975.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4463.46,6801.89");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4817.11,6900.43");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5803.45,6948.58");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=506.47,6614.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1001.40,6708.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1666.64,6649.16");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2957.30,6583.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3933.02,6664.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5158.13,6751.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1075.68,6297.73");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1905.06,6350.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3339.83,6514.26");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4465.37,6328.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5164.65,6365.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=481.44,6182.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1478.10,6102.33");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2752.75,6230.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3584.17,6094.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4882.14,6135.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5662.11,6164.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=95.02,5943.90");
    }

}

