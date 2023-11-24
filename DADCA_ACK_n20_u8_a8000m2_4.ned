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

network DADCA_ACK_n20_u8_a8000m2_4
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
        @display("i=misc/sensor;p=841.80,7889.80");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1365.42,7792.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2444.79,7837.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3756.69,7914.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4681.12,7790.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4977.86,7994.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=53.67,7584.68");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=480.93,7716.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=776.90,7524.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1897.45,7650.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3157.79,7594.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5270.36,7741.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5674.03,7693.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1236.33,7316.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1759.35,7262.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2627.80,7454.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2988.03,7272.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3675.28,7503.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4548.68,7429.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5008.45,7319.14");
    }

}

