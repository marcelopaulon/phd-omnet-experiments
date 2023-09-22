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

network DADCA_ACK_n20_a8000m2_3
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
        @display("i=misc/sensor;p=182.37,7883.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2299.09,7932.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3389.32,7932.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3797.35,7919.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4175.88,7775.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4701.48,7889.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5219.15,7951.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5601.33,7815.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1017.45,7721.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1321.17,7513.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1693.20,7655.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2702.58,7715.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5987.65,7689.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=997.23,7259.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2121.34,7330.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3257.24,7369.32");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3774.68,7419.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4446.00,7401.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5153.15,7332.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=389.33,7200.94");
    }

}

