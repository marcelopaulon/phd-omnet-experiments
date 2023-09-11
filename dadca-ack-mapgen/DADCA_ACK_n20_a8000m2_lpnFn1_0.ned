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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=105.84,180.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=142.42,253.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=56.24,255.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=212.58,227.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=202.89,292.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=259.17,266.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=321.67,225.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=247.97,383.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=224.65,121.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=296.28,328.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=430.28,217.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=418.56,146.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=335.31,152.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=100.07,323.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=9.03,311.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=168.95,385.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=360.05,301.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=375.63,360.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=452.92,318.66");
    }

}

