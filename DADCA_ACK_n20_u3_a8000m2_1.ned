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

network DADCA_ACK_n20_u3_a8000m2_1
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
        @display("i=misc/sensor;p=1489.74,7907.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2724.00,7900.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3551.58,7804.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4528.34,7810.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4903.82,7857.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5312.38,7864.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5706.06,7887.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1848.08,7560.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3915.89,7618.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1087.41,7441.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2536.99,7440.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3015.59,7288.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3566.57,7364.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4215.44,7360.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4611.53,7362.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5104.45,7286.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5430.79,7501.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5836.60,7281.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1531.77,7149.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2256.20,7151.06");
    }

}

