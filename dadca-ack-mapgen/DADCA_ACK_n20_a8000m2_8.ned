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

network DADCA_ACK_n20_a8000m2_8
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
        @display("i=misc/sensor;p=85.76,7961.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=304.83,7996.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=706.22,7972.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=770.35,7985.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=831.04,7970.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=945.50,7978.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=176.68,7936.24");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=368.95,7945.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=433.04,7931.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=507.51,7946.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=600.00,7922.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=67.15,7903.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=200.14,7872.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=257.64,7908.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=500.51,7876.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=673.99,7900.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=731.71,7873.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=793.79,7904.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=861.34,7875.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=949.38,7903.48");
    }

}

