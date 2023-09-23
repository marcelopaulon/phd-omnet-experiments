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
        @display("i=misc/sensor;p=1639.90,7869.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3321.56,7850.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4797.38,7780.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5369.02,7966.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=602.69,7578.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1258.43,7587.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1799.61,7507.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2733.65,7748.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3077.59,7524.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4160.67,7716.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5644.20,7667.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2280.53,7346.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4007.59,7271.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4652.87,7424.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5037.27,7475.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5423.99,7302.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2821.34,7251.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3390.94,7241.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5879.65,7021.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=489.17,6868.68");
    }

}

