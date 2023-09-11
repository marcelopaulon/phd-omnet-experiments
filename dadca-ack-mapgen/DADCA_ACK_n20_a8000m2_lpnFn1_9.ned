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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=11.20,111.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=25.42,50.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=33.64,183.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=26.35,248.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=33.50,342.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=6.44,427.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=54.68,467.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=129.76,475.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=181.91,408.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=268.60,492.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=226.44,572.91");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=199.12,517.83");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=67.28,403.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=309.60,386.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=329.94,483.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=298.02,314.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=95.86,529.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=34.58,600.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=25.17,692.56");
    }

}

