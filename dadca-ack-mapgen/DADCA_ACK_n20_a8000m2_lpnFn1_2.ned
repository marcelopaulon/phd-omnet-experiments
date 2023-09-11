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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=87.92,12.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=212.18,75.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=207.48,161.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=258.28,126.92");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=279.46,223.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=357.10,174.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=331.44,109.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=396.82,97.39");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=352.20,53.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=271.72,62.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=424.27,240.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=514.46,287.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=510.66,350.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=514.51,427.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=430.89,424.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=452.78,486.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=400.61,558.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=410.05,654.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=454.88,599.03");
    }

}

