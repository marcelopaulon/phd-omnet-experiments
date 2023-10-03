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

network DADCA_ACK_n20_u2_a8000m2_0
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
        @display("i=misc/sensor;p=1538.03,7755.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2223.95,7935.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2686.28,7900.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3468.90,7752.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4151.89,7903.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1986.19,7561.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2971.63,7553.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4738.57,7627.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5357.27,7701.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5918.71,7559.47");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2616.68,7429.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3716.19,7427.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4246.35,7372.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5034.15,7339.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1577.82,7102.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2251.96,7230.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3147.11,7181.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5383.20,7163.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2006.51,6862.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2570.08,6989.69");
    }

}

