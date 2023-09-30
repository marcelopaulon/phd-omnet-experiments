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

network DADCA_ACK_n50_u16_a8000m2_1
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
        @display("i=misc/sensor;p=1369.12,7904.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2322.67,7917.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4532.79,7984.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4903.18,7958.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1282.99,7564.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2177.54,7572.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2843.03,7664.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3480.60,7742.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4774.30,7511.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5226.48,7698.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5883.04,7716.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=813.52,7349.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3852.82,7289.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4379.31,7474.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1725.00,7110.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2728.21,7162.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3225.03,7129.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4499.59,7060.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5070.38,7169.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5639.37,7056.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1125.67,6959.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2181.79,6764.53");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3933.03,6941.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=950.17,6646.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1452.85,6573.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2666.19,6612.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3022.25,6749.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3594.52,6716.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5086.85,6706.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5996.95,6563.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1999.14,6338.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2491.30,6305.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2939.65,6288.51");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4551.07,6347.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=92.70,6155.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=696.36,6120.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1389.82,6197.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3463.74,6241.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4054.16,6242.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4768.92,6049.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5163.88,6208.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5850.96,6087.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1577.93,5839.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2443.63,5849.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2887.14,5902.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3944.26,5888.86");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=484.20,5755.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=874.46,5724.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2130.18,5640.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3232.18,5700.67");
    }

}

