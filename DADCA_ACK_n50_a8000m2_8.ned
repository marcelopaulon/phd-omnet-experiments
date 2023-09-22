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

network DADCA_ACK_n50_a8000m2_8
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
        @display("i=misc/sensor;p=774.63,7817.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1765.90,7933.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2500.90,7870.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2902.86,7984.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4532.53,7987.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5312.51,7778.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=222.25,7614.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1118.48,7670.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2110.24,7513.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3613.81,7747.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3946.13,7560.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4415.86,7557.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4840.48,7671.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5788.75,7603.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=15.89,7284.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1694.45,7480.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2655.18,7348.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3296.78,7449.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5472.05,7323.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=795.19,7161.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1239.86,7205.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2250.12,7128.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4186.57,7200.89");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4811.31,7253.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=383.33,6988.27");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=989.60,6858.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1420.44,6883.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1867.27,6996.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2903.15,6785.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3288.67,6820.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3930.30,6861.27");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4542.31,6799.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5145.64,6793.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5562.25,6842.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3.76,6531.47");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=561.13,6670.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1511.93,6544.81");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2513.03,6755.89");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3753.46,6523.13");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=896.36,6369.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1879.48,6408.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2923.60,6345.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5052.95,6371.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5502.83,6291.97");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5935.59,6382.08");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=461.73,6232.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2260.30,6222.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3375.26,6098.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4380.66,6244.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4769.63,6058.60");
    }

}

