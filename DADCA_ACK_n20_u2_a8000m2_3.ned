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

network DADCA_ACK_n20_u2_a8000m2_3
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
        @display("i=misc/sensor;p=438.10,7835.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1364.24,7814.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2090.98,7828.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3349.63,7902.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4427.58,7922.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5784.14,7993.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=877.85,7628.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1619.03,7536.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2460.80,7574.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3870.43,7747.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4915.51,7739.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5520.34,7741.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=679.70,7310.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2120.40,7474.84");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2991.50,7336.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3597.66,7399.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4057.79,7431.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4659.97,7399.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5697.74,7411.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1288.26,7181.69");
    }

}

