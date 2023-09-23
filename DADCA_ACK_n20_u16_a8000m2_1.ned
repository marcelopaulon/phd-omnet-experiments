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

network DADCA_ACK_n20_u16_a8000m2_1
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
        @display("i=misc/sensor;p=1718.84,7932.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2207.00,7887.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2924.50,7890.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3554.19,7929.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4373.58,7942.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5766.95,7781.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1588.47,7579.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1954.27,7607.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3312.85,7585.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3815.68,7610.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4795.34,7742.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5336.74,7663.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=709.22,7500.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2656.20,7401.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4224.14,7356.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4697.21,7386.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=296.35,7196.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=874.67,7046.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1247.84,7243.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1634.39,7166.27");
    }

}

