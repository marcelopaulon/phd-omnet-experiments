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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=27.89,138.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=35.64,74.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=106.13,185.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=121.73,249.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=77.24,289.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=79.96,382.68");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=21.48,437.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=151.21,317.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=220.17,337.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=262.30,257.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=297.04,373.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=363.85,443.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=369.54,505.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=435.23,565.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=491.43,542.37");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=534.22,589.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=468.07,639.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=461.89,410.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=303.91,498.39");
    }

}

