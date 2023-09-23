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

network DADCA_ACK_n20_u3_a8000m2_4
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
        @display("i=misc/sensor;p=345.79,7822.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1104.83,7899.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1794.15,7912.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2361.16,7766.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2700.91,7986.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3130.35,7862.13");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3532.91,7978.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4316.81,7883.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5562.14,7877.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=58.47,7540.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=800.39,7543.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2942.89,7554.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4079.92,7588.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4833.76,7640.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1461.47,7470.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1819.00,7449.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3421.22,7446.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4468.41,7495.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5434.06,7284.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5935.42,7402.17");
    }

}

