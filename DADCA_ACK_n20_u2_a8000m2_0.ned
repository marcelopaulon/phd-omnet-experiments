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
        @display("i=misc/sensor;p=3690.37,7772.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4883.94,7876.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=866.86,7595.86");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1358.05,7726.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1935.44,7732.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3222.51,7610.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5375.22,7664.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=455.09,7491.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1655.04,7478.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2471.32,7357.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4127.92,7453.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4581.57,7289.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=558.13,7074.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1256.06,7071.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1923.29,7145.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2267.01,7068.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2674.61,7033.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3583.77,7029.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4164.79,7092.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5637.95,7222.33");
    }

}

