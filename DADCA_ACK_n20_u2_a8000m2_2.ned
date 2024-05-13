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

network DADCA_ACK_n20_u2_a8000m2_2
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
        @display("i=misc/sensor;p=1428.63,7903.70");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2685.15,7932.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3645.75,7775.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3971.70,7977.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4660.09,7978.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5099.52,7876.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5622.20,7967.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=321.14,7682.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=686.83,7584.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1078.32,7645.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1646.26,7602.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2335.59,7583.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3255.92,7728.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4142.86,7560.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4518.78,7581.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5411.47,7682.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5977.98,7512.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=363.72,7307.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3817.54,7333.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5005.94,7369.47");
    }

}

