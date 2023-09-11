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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=34.77,115.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=116.81,213.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=45.83,183.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=80.61,273.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=107.03,330.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=151.24,418.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=161.19,298.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=241.14,243.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=303.09,213.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=165.29,143.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=189.83,204.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=318.89,301.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=215.83,415.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=305.28,426.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=280.40,498.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=351.81,537.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=400.06,306.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=416.65,230.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=422.56,142.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=201.41,48.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=276.53,99.18");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=330.74,52.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=398.18,5.70");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=508.90,106.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=513.65,33.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=352.39,112.83");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=451.51,45.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=599.29,98.53");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=582.19,172.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=635.84,202.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=496.51,169.44");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=542.86,234.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=588.69,320.56");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=679.21,289.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=719.09,376.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=621.47,373.51");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=601.94,447.10");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=720.10,225.26");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=739.38,296.30");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=797.01,156.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=796.03,259.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=893.91,257.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=850.55,330.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=556.00,371.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=509.92,456.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=466.09,403.55");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=444.27,480.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=236.45,556.22");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=301.81,604.50");
    }

}

